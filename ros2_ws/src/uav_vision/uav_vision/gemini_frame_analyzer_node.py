#!/usr/bin/env python3

import io
import json
import os
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from uav_interfaces.msg import GeminiReport
from uav_vision.common.gemini_utils import append_error
from uav_vision.common.gemini_utils import as_bool
from uav_vision.common.gemini_utils import as_target_candidates
from uav_vision.common.gemini_utils import find_candidate
from uav_vision.common.gemini_utils import gimbal_preset_for_candidate
from uav_vision.common.gemini_utils import make_request_id
from uav_vision.common.gemini_utils import parse_json_response
from uav_vision.common.gemini_utils import select_primary_candidate_index
from uav_vision.common.gemini_prompts import DEFAULT_PROMPT_VERSION
from uav_vision.common.gemini_prompts import get_prompt
from uav_vision.common.image_utils import image_msg_to_pil


class GeminiFrameAnalyzerNode(Node):
    """
    - Subscribe to either raw camera frames or selector output frames.
    - Periodically send the latest frame to Gemini using a fixed JSON prompt.
    - Publish parsed GeminiReport messages and persist raw JSON reports to disk.
    """

    def __init__(self):
        super().__init__("gemini_frame_analyzer")

        self.declare_parameter("image_topic", "/uav/vision/selected_frame/image_raw")
        self.declare_parameter("report_topic", "/uav/vision/gemini_report")
        self.declare_parameter("model", "gemini-2.5-flash")
        self.declare_parameter("analysis_period_sec", 5.0)
        self.declare_parameter("analysis_mode", "periodic")
        self.declare_parameter("trigger_topic", "/uav/vision/analyze_trigger")
        self.declare_parameter("max_calls", 0)  # 0: unlimited
        self.declare_parameter("prompt_version", DEFAULT_PROMPT_VERSION)
        self.declare_parameter("target_query", "person")
        self.declare_parameter("max_width", 640)
        self.declare_parameter("jpeg_quality", 70)
        self.declare_parameter("save_reports", True)
        self.declare_parameter("report_dir", os.path.expanduser("~/uav_experiments/gemini_reports"))

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.report_topic = str(self.get_parameter("report_topic").value)
        self.model = str(self.get_parameter("model").value)
        self.analysis_period_sec = float(self.get_parameter("analysis_period_sec").value)
        self.analysis_mode = str(self.get_parameter("analysis_mode").value).strip().lower()
        self.trigger_topic = str(self.get_parameter("trigger_topic").value)
        self.max_calls = max(int(self.get_parameter("max_calls").value), 0)
        self.prompt_version = str(self.get_parameter("prompt_version").value)
        self.target_query = str(self.get_parameter("target_query").value).strip()
        self.analysis_prompt, self.prompt_version = get_prompt(
            self.prompt_version,
            self.target_query,
        )
        self.max_width = int(self.get_parameter("max_width").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.save_reports = bool(self.get_parameter("save_reports").value)
        self.report_dir = os.path.expanduser(str(self.get_parameter("report_dir").value))

        self.latest_msg = None
        self.latest_lock = threading.Lock()
        self.inflight = False
        self.report_index = 0

        if self.analysis_mode not in {"periodic", "event", "both", "on_image"}:
            raise ValueError(
                "analysis_mode must be 'periodic', 'event', 'both', or 'on_image'."
            )

        self.client = self.make_client()
        self.create_subscription(Image, self.image_topic, self.on_image, 10)
        if self.analysis_mode in {"event", "both"}:
            self.create_subscription(Empty, self.trigger_topic, self.on_trigger, 10)
        self.report_pub = self.create_publisher(GeminiReport, self.report_topic, 10)
        self.timer = None
        if self.analysis_mode in {"periodic", "both"}:
            self.timer = self.create_timer(self.analysis_period_sec, self.on_timer)

        if self.save_reports:
            os.makedirs(self.report_dir, exist_ok=True)

        self.get_logger().warn(
            f"Gemini frame analyzer started: image={self.image_topic}, "
            f"report={self.report_topic}, model={self.model}, "
            f"prompt={self.prompt_version}, target={self.target_query!r}, "
            f"mode={self.analysis_mode}, "
            f"trigger={self.trigger_topic}, period={self.analysis_period_sec}s, "
            f"max_calls={self.max_calls}"
        )

    def make_client(self):
        if not os.getenv("GEMINI_API_KEY"):
            self.get_logger().error(
                "GEMINI_API_KEY is not set. Gemini analyzer will not call API."
            )
            return None

        try:
            from google import genai
        except ImportError:
            self.get_logger().error("Missing google-genai. Install/rebuild Docker image first.")
            return None

        return genai.Client()

    def on_image(self, msg):
        with self.latest_lock:
            self.latest_msg = msg
        if self.analysis_mode == "on_image":
            self.request_analysis(source="image", msg=msg)

    def on_timer(self):
        self.request_analysis(source="timer")

    def on_trigger(self, _msg):
        self.request_analysis(source="trigger")

    def request_analysis(self, *, source: str, msg=None):
        """Start Gemini analysis for the latest image when the analyzer is idle."""

        if self.client is None or self.inflight:
            return
        if self.call_budget_exhausted():
            self.get_logger().warn(
                f"Gemini call budget exhausted: {self.report_index}/{self.max_calls}",
                throttle_duration_sec=10.0,
            )
            return

        if msg is None:
            with self.latest_lock:
                msg = self.latest_msg

        if msg is None:
            self.get_logger().warn("No image received yet.", throttle_duration_sec=10.0)
            return

        self.inflight = True
        threading.Thread(target=self.analyze_frame, args=(msg,), daemon=True).start()

    def call_budget_exhausted(self):
        """Return true when the configured Gemini call budget has been consumed."""

        return self.max_calls > 0 and self.report_index >= self.max_calls

    def analyze_frame(self, msg):
        """Send one image frame to Gemini and publish a structured GeminiReport."""

        started_at = time.monotonic()
        raw_text = ""
        parsed = None
        error_message = ""
        call_index = self.report_index + 1
        request_id = make_request_id(msg, call_index)

        try:
            image = image_msg_to_pil(msg)
            if image.width > self.max_width:
                new_height = int(image.height * (self.max_width / image.width))
                image = image.resize((self.max_width, new_height))

            buffer = io.BytesIO()
            image.save(buffer, format="JPEG", quality=self.jpeg_quality)
            buffer.seek(0)

            from PIL import Image as PILImage

            jpeg_image = PILImage.open(buffer)
            response = self.client.models.generate_content(
                model=self.model,
                contents=[self.analysis_prompt, jpeg_image],
            )

            raw_text = (response.text or "").strip()
            parsed = parse_json_response(raw_text)
            if not isinstance(parsed, dict):
                error_message = "Gemini response was not valid JSON."
        except Exception as exc:
            error_message = str(exc)
            self.get_logger().error(f"Gemini frame analysis failed: {exc}")
        finally:
            latency_sec = time.monotonic() - started_at
            report = self.make_report_msg(
                msg=msg,
                raw_text=raw_text,
                parsed=parsed,
                latency_sec=latency_sec,
                error_message=error_message,
                request_id=request_id,
                call_index=call_index,
            )

            if self.save_reports:
                try:
                    self.write_report_file(report)
                except Exception as exc:
                    report.error_message = append_error(
                        report.error_message,
                        f"Failed to save report: {exc}",
                    )
                    self.get_logger().error(report.error_message)

            self.report_pub.publish(report)
            self.report_index += 1
            self.inflight = False

    def make_report_msg(
        self,
        *,
        msg,
        raw_text,
        parsed,
        latency_sec,
        error_message,
        request_id,
        call_index,
    ):
        report = GeminiReport()
        report.header = msg.header
        report.request_id = request_id
        report.call_index = call_index
        report.prompt_version = self.prompt_version
        report.image_source_topic = self.image_topic
        report.model = self.model
        report.latency_sec = float(latency_sec)
        report.parsed_ok = isinstance(parsed, dict)
        report.error_message = error_message
        report.primary_candidate_index = -1
        report.recommended_gimbal_preset = "HOLD"
        report.target_query = self.target_query

        if isinstance(parsed, dict):
            report.scene_summary = str(parsed.get("scene_summary", ""))
            report.target_candidates = as_target_candidates(
                parsed.get("target_candidates", [])
            )
            for candidate in report.target_candidates:
                candidate.target_label = self.target_query
            report.target_detected = (
                as_bool(parsed.get("target_detected", False))
                or bool(report.target_candidates)
            )
            report.primary_candidate_index = select_primary_candidate_index(
                parsed.get("primary_candidate_index", -1),
                report.target_candidates,
            )
            primary_candidate = find_candidate(
                report.target_candidates,
                report.primary_candidate_index,
            )
            report.recommended_gimbal_preset = gimbal_preset_for_candidate(
                primary_candidate
            )

        report.raw_json = raw_text
        return report

    def write_report_file(self, report):
        timestamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(
            self.report_dir,
            f"gemini_frame_report_{timestamp}_{report.call_index:04d}.json",
        )
        report.report_path = path
        payload = {
            "stamp": {
                "sec": int(report.header.stamp.sec),
                "nanosec": int(report.header.stamp.nanosec),
            },
            "request_id": report.request_id,
            "call_index": report.call_index,
            "prompt_version": report.prompt_version,
            "image_source_topic": report.image_source_topic,
            "report_path": report.report_path,
            "model": report.model,
            "latency_sec": report.latency_sec,
            "parsed_ok": report.parsed_ok,
            "error_message": report.error_message,
            "scene_summary": report.scene_summary,
            "target_query": report.target_query,
            "target_detected": report.target_detected,
            "primary_candidate_index": report.primary_candidate_index,
            "recommended_gimbal_preset": report.recommended_gimbal_preset,
            "target_candidates": [
                {
                    "candidate_index": candidate.candidate_index,
                    "target_label": candidate.target_label,
                    "confidence": candidate.confidence,
                    "bbox_norm": {
                        "x_min": candidate.bbox_x_min_norm,
                        "y_min": candidate.bbox_y_min_norm,
                        "x_max": candidate.bbox_x_max_norm,
                        "y_max": candidate.bbox_y_max_norm,
                    },
                    "distance_bucket": candidate.distance_bucket,
                }
                for candidate in report.target_candidates
            ],
            "raw_json": report.raw_json,
        }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)

        return path


def main(args=None):
    rclpy.init(args=args)
    node = GeminiFrameAnalyzerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
