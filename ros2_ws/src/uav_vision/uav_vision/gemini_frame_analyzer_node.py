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
from uav_interfaces.msg import GeminiReport


ANALYSIS_PROMPT = """
You are analyzing an image from a UAV front camera.

Return JSON only.

Schema:
{
  "scene_summary": "short description",
  "visible_objects": ["object1", "object2"],
  "possible_targets": ["target candidate list"],
  "hazards": ["hazard list"],
  "confidence": 0.0,
  "recommended_action": "continue | inspect | return_home | unknown",
  "need_gimbal_adjustment": false,
  "gimbal_direction": "up | down | hold | unknown"
}

Be conservative. If there is no clear target, say no clear target.
Gemini only provides perception and intent; do not output actuator commands.
"""


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
        self.declare_parameter("max_width", 640)
        self.declare_parameter("jpeg_quality", 70)
        self.declare_parameter("save_reports", True)
        self.declare_parameter("report_dir", os.path.expanduser("~/uav_experiments/gemini_reports"))

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.report_topic = str(self.get_parameter("report_topic").value)
        self.model = str(self.get_parameter("model").value)
        self.analysis_period_sec = float(self.get_parameter("analysis_period_sec").value)
        self.max_width = int(self.get_parameter("max_width").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.save_reports = bool(self.get_parameter("save_reports").value)
        self.report_dir = os.path.expanduser(str(self.get_parameter("report_dir").value))

        self.latest_msg = None
        self.latest_lock = threading.Lock()
        self.inflight = False
        self.report_index = 0

        self.client = self.make_client()
        self.create_subscription(Image, self.image_topic, self.on_image, 10)
        self.report_pub = self.create_publisher(GeminiReport, self.report_topic, 10)
        self.timer = self.create_timer(self.analysis_period_sec, self.on_timer)

        if self.save_reports:
            os.makedirs(self.report_dir, exist_ok=True)

        self.get_logger().warn(
            f"Gemini frame analyzer started: image={self.image_topic}, "
            f"report={self.report_topic}, model={self.model}, "
            f"period={self.analysis_period_sec}s"
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

    def on_timer(self):
        if self.client is None or self.inflight:
            return

        with self.latest_lock:
            msg = self.latest_msg

        if msg is None:
            self.get_logger().warn("No image received yet.", throttle_duration_sec=10.0)
            return

        self.inflight = True
        threading.Thread(target=self.analyze_frame, args=(msg,), daemon=True).start()

    def analyze_frame(self, msg):
        started_at = time.monotonic()
        raw_text = ""
        parsed = None
        image_path = ""

        try:
            image = self.image_msg_to_pil(msg)
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
                contents=[ANALYSIS_PROMPT, jpeg_image],
            )

            raw_text = (response.text or "").strip()
            parsed = self.parse_json_response(raw_text)
        except Exception as exc:
            self.get_logger().error(f"Gemini frame analysis failed: {exc}")
        finally:
            latency_sec = time.monotonic() - started_at
            report = self.make_report_msg(
                msg=msg,
                raw_text=raw_text,
                parsed=parsed,
                latency_sec=latency_sec,
                image_path=image_path,
            )

            if self.save_reports:
                image_path = self.write_report_file(report)
                report.image_path = image_path

            self.report_pub.publish(report)
            self.report_index += 1
            self.inflight = False

    def make_report_msg(self, *, msg, raw_text, parsed, latency_sec, image_path):
        report = GeminiReport()
        report.header = msg.header
        report.image_source_topic = self.image_topic
        report.image_path = image_path
        report.model = self.model
        report.latency_sec = float(latency_sec)
        report.parsed_ok = isinstance(parsed, dict)

        if isinstance(parsed, dict):
            report.scene_summary = str(parsed.get("scene_summary", ""))
            report.visible_objects = self.as_string_list(parsed.get("visible_objects", []))
            report.possible_targets = self.as_string_list(parsed.get("possible_targets", []))
            report.hazards = self.as_string_list(parsed.get("hazards", []))
            report.confidence = float(parsed.get("confidence", 0.0) or 0.0)
            report.recommended_action = str(parsed.get("recommended_action", "unknown"))
            report.need_gimbal_adjustment = bool(parsed.get("need_gimbal_adjustment", False))
            report.gimbal_direction = str(parsed.get("gimbal_direction", "unknown"))

        report.raw_json = raw_text
        return report

    def write_report_file(self, report):
        timestamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(
            self.report_dir,
            f"gemini_frame_report_{timestamp}_{self.report_index:04d}.json",
        )
        payload = {
            "stamp": {
                "sec": int(report.header.stamp.sec),
                "nanosec": int(report.header.stamp.nanosec),
            },
            "image_source_topic": report.image_source_topic,
            "model": report.model,
            "latency_sec": report.latency_sec,
            "parsed_ok": report.parsed_ok,
            "scene_summary": report.scene_summary,
            "visible_objects": list(report.visible_objects),
            "possible_targets": list(report.possible_targets),
            "hazards": list(report.hazards),
            "confidence": report.confidence,
            "recommended_action": report.recommended_action,
            "need_gimbal_adjustment": report.need_gimbal_adjustment,
            "gimbal_direction": report.gimbal_direction,
            "raw_json": report.raw_json,
        }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)

        return path

    @staticmethod
    def parse_json_response(raw_text):
        text = raw_text.strip()
        if text.startswith("```"):
            lines = text.splitlines()
            if lines and lines[0].startswith("```"):
                lines = lines[1:]
            if lines and lines[-1].startswith("```"):
                lines = lines[:-1]
            text = "\n".join(lines).strip()

        try:
            return json.loads(text)
        except json.JSONDecodeError:
            return None

    @staticmethod
    def as_string_list(value):
        if not isinstance(value, list):
            return []
        return [str(item) for item in value]

    @staticmethod
    def image_msg_to_pil(msg):
        try:
            from PIL import Image as PILImage
        except ImportError as exc:
            raise RuntimeError("Missing Pillow. Install/rebuild Docker image first.") from exc

        encoding = msg.encoding.lower()
        data = bytes(msg.data)

        formats = {
            "rgb8": ("RGB", "RGB", 3),
            "bgr8": ("RGB", "BGR", 3),
            "rgba8": ("RGBA", "RGBA", 4),
            "bgra8": ("RGBA", "BGRA", 4),
            "mono8": ("L", "L", 1),
        }
        if encoding not in formats:
            raise RuntimeError(f"Unsupported image encoding: {msg.encoding}")

        mode, raw_mode, bytes_per_pixel = formats[encoding]
        expected_step = msg.width * bytes_per_pixel
        if msg.step < expected_step:
            raise RuntimeError(f"Invalid image step: step={msg.step}, expected>={expected_step}")

        if msg.step == expected_step:
            raw = data[:expected_step * msg.height]
        else:
            rows = []
            for row in range(msg.height):
                start = row * msg.step
                rows.append(data[start:start + expected_step])
            raw = b"".join(rows)

        return PILImage.frombytes(mode, (msg.width, msg.height), raw, "raw", raw_mode)


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
