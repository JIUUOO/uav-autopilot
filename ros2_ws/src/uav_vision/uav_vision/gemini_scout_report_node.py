#!/usr/bin/env python3

import io
import os
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from uav_vision.common.image_utils import image_msg_to_pil


REPORT_PROMPT = """You are observing a UAV camera frame for a flight test.
Report in Korean, in a concise military field-report style.
Only describe visible facts and likely safety-relevant anomalies.
Do not recommend flight-control commands.

Format:
보고:
- 상황:
- 특이사항:
- 위험도: LOW/MEDIUM/HIGH
"""


class GeminiScoutReportNode(Node):
    def __init__(self):
        super().__init__("gemini_scout_report")

        self.declare_parameter("image_topic", "/uav/camera/front/image_raw")
        self.declare_parameter("model", "gemini-3.5-flash")
        self.declare_parameter("report_period_sec", 5.0)
        self.declare_parameter("max_width", 640)
        self.declare_parameter("jpeg_quality", 70)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.model = str(self.get_parameter("model").value)
        self.report_period_sec = float(self.get_parameter("report_period_sec").value)
        self.max_width = int(self.get_parameter("max_width").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        self.latest_msg = None
        self.latest_lock = threading.Lock()
        self.inflight = False

        self.client = self._make_client()
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.on_image,
            10,
        )
        self.timer = self.create_timer(self.report_period_sec, self.on_timer)

        self.get_logger().warn(
            f"Gemini scout report started: topic={self.image_topic}, "
            f"model={self.model}, period={self.report_period_sec}s"
        )

    def _make_client(self):
        if not os.getenv("GEMINI_API_KEY"):
            self.get_logger().error(
                "GEMINI_API_KEY is not set. Gemini report node will not call API."
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
        threading.Thread(target=self.run_report, args=(msg,), daemon=True).start()

    def run_report(self, msg):
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
                contents=[REPORT_PROMPT, jpeg_image],
            )

            text = (response.text or "").strip()
            if text:
                self.get_logger().warn(text)
            else:
                self.get_logger().warn("Gemini returned an empty report.")
        except Exception as exc:
            self.get_logger().error(f"Gemini scout report failed: {exc}")
        finally:
            self.inflight = False

def main(args=None):
    rclpy.init(args=args)
    node = GeminiScoutReportNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
