#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from std_msgs.msg import UInt16
from uav_interfaces.msg import GeminiReport


class GimbalScanFsmNode(Node):
    """
    - Rule-based fixed scan; Gemini does not adapt the order or number of presets.
    - Publish calibrated gimbal PWM presets in a deterministic scan sequence.
    - Wait for mechanical settle time after each preset.
    - Request a fresh frame-selection window once the camera viewpoint should be stable.
    - Wait for the resulting Gemini report before advancing to the next preset.
    """

    def __init__(self):
        super().__init__("gimbal_scan_fsm")

        self.declare_parameter("pitch_target_topic", "/uav/gimbal/pitch_target_pwm")
        self.declare_parameter("trigger_topic", "/uav/vision/analyze_trigger")
        self.declare_parameter("selection_trigger_topic", "/uav/vision/select_frame_trigger")
        self.declare_parameter("selected_image_topic", "/uav/vision/selected_frame/image_raw")
        self.declare_parameter("gemini_report_topic", "/uav/vision/gemini_report")
        self.declare_parameter("capture_mode", "selector_event")
        self.declare_parameter("preset_far_pwm", 1520)
        self.declare_parameter("preset_near_pwm", 1580)
        self.declare_parameter("scan_sequence", "PRESET_FAR,PRESET_NEAR")
        self.declare_parameter("settle_sec", 1.0)
        self.declare_parameter("post_trigger_sec", 0.2)
        self.declare_parameter("analysis_timeout_sec", 30.0)
        self.declare_parameter("scan_count", 1)  # How many times to repeat the full gimbal scan sequence.
        self.declare_parameter("auto_start", True)
        self.declare_parameter("timer_hz", 10.0)

        self.pitch_target_topic = str(self.get_parameter("pitch_target_topic").value)
        self.trigger_topic = str(self.get_parameter("trigger_topic").value)
        self.selection_trigger_topic = str(
            self.get_parameter("selection_trigger_topic").value
        )
        self.selected_image_topic = str(self.get_parameter("selected_image_topic").value)
        self.gemini_report_topic = str(self.get_parameter("gemini_report_topic").value)
        self.capture_mode = str(self.get_parameter("capture_mode").value).strip().lower()
        self.preset_far_pwm = int(self.get_parameter("preset_far_pwm").value)
        self.preset_near_pwm = int(self.get_parameter("preset_near_pwm").value)
        self.scan_sequence = self.parse_sequence(
            str(self.get_parameter("scan_sequence").value)
        )
        self.settle_sec = max(float(self.get_parameter("settle_sec").value), 0.0)
        self.post_trigger_sec = max(
            float(self.get_parameter("post_trigger_sec").value),
            0.0,
        )
        self.analysis_timeout_sec = max(
            float(self.get_parameter("analysis_timeout_sec").value),
            0.1,
        )
        self.scan_count = max(int(self.get_parameter("scan_count").value), 0)
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.timer_hz = max(float(self.get_parameter("timer_hz").value), 0.1)

        self.pitch_pub = self.create_publisher(UInt16, self.pitch_target_topic, 10)
        self.trigger_pub = self.create_publisher(Empty, self.trigger_topic, 10)
        self.selection_trigger_pub = self.create_publisher(
            Empty,
            self.selection_trigger_topic,
            10,
        )
        self.create_subscription(
            Image,
            self.selected_image_topic,
            self.on_selected_image,
            10,
        )
        self.create_subscription(
            GeminiReport,
            self.gemini_report_topic,
            self.on_gemini_report,
            10,
        )

        if self.capture_mode not in {"selector_event", "direct_trigger"}:
            raise ValueError("capture_mode must be 'selector_event' or 'direct_trigger'.")

        self.active = False
        self.state = "IDLE"
        self.sequence_index = 0
        self.completed_scans = 0
        self.latest_report_call_index = 0
        self.waiting_after_call_index = 0
        self.expected_report_stamp = None
        self.received_report_stamps = set()
        self.state_started_at = time.monotonic()

        self.timer = self.create_timer(1.0 / self.timer_hz, self.on_timer)

        if self.auto_start:
            self.start_scan()

        self.get_logger().warn(
            "Gimbal scan FSM started: "
            f"sequence={self.scan_sequence}, scan_count={self.scan_count}, "
            f"settle_sec={self.settle_sec}, capture_mode={self.capture_mode}, "
            f"analysis_timeout_sec={self.analysis_timeout_sec}"
        )

    def start_scan(self):
        """Start the configured gimbal scan sequence."""

        if not self.scan_sequence:
            self.get_logger().error("scan_sequence is empty.")
            return

        self.active = True
        self.state = "COMMAND_PRESET"
        self.sequence_index = 0
        self.completed_scans = 0
        self.state_started_at = time.monotonic()

    def on_timer(self):
        if not self.active:
            return

        now = time.monotonic()
        if self.state == "COMMAND_PRESET":
            self.publish_current_preset()
            self.state = "SETTLE"
            self.state_started_at = now
            return

        if self.state == "SETTLE":
            if now - self.state_started_at >= self.settle_sec:
                if self.request_capture_and_analysis():
                    self.state = (
                        "WAIT_SELECTED_FRAME"
                        if self.capture_mode == "selector_event"
                        else "WAIT_REPORT"
                    )
                    self.state_started_at = now
            return

        if self.state == "WAIT_SELECTED_FRAME":
            if now - self.state_started_at >= self.analysis_timeout_sec:
                self.get_logger().error(
                    f"Selected frame timeout after {self.current_preset_name()}."
                )
                self.active = False
                self.state = "ERROR"
            return

        if self.state == "WAIT_REPORT":
            if self.expected_report_received():
                self.get_logger().warn(
                    f"Gemini report received for {self.current_preset_name()}: "
                    f"call_index={self.latest_report_call_index}"
                )
                self.state = "POST_TRIGGER"
                self.state_started_at = now
            elif now - self.state_started_at >= self.analysis_timeout_sec:
                self.get_logger().error(
                    f"Gemini report timeout after {self.current_preset_name()}."
                )
                self.active = False
                self.state = "ERROR"
            return

        if self.state == "POST_TRIGGER":
            if now - self.state_started_at >= self.post_trigger_sec:
                self.advance_sequence()

    def request_capture_and_analysis(self):
        self.waiting_after_call_index = self.latest_report_call_index
        self.expected_report_stamp = None
        if self.capture_mode == "selector_event":
            if self.selection_trigger_pub.get_subscription_count() < 1:
                self.get_logger().warn(
                    "Waiting for frame selector trigger subscription.",
                    throttle_duration_sec=2.0,
                )
                return False
            self.selection_trigger_pub.publish(Empty())
            self.get_logger().warn(
                f"Fresh frame selection requested after {self.current_preset_name()}."
            )
            return True

        if self.trigger_pub.get_subscription_count() < 1:
            self.get_logger().warn(
                "Waiting for Gemini trigger subscription.",
                throttle_duration_sec=2.0,
            )
            return False
        self.trigger_pub.publish(Empty())
        self.get_logger().warn(
            f"Gemini direct trigger published after {self.current_preset_name()}."
        )
        return True

    def on_gemini_report(self, msg):
        self.latest_report_call_index = max(
            self.latest_report_call_index,
            int(msg.call_index),
        )
        self.received_report_stamps.add(self.stamp_tuple(msg.header.stamp))

    def on_selected_image(self, msg):
        if self.state != "WAIT_SELECTED_FRAME":
            return

        self.expected_report_stamp = self.stamp_tuple(msg.header.stamp)
        self.state = "WAIT_REPORT"
        self.state_started_at = time.monotonic()
        self.get_logger().warn(
            f"Selected frame received for {self.current_preset_name()}: "
            f"stamp={self.expected_report_stamp}"
        )

    def expected_report_received(self):
        if self.capture_mode == "selector_event":
            return (
                self.expected_report_stamp is not None
                and self.expected_report_stamp in self.received_report_stamps
            )
        return self.latest_report_call_index > self.waiting_after_call_index

    @staticmethod
    def stamp_tuple(stamp):
        return int(stamp.sec), int(stamp.nanosec)

    def publish_current_preset(self):
        preset_name = self.current_preset_name()
        pwm = self.pwm_for_preset(preset_name)
        msg = UInt16()
        msg.data = pwm
        self.pitch_pub.publish(msg)
        self.get_logger().warn(f"Gimbal preset command: {preset_name} pwm={pwm}")

    def advance_sequence(self):
        self.sequence_index += 1
        if self.sequence_index < len(self.scan_sequence):
            self.state = "COMMAND_PRESET"
            self.state_started_at = time.monotonic()
            return

        self.completed_scans += 1
        if self.scan_count == 0 or self.completed_scans < self.scan_count:
            self.sequence_index = 0
            self.state = "COMMAND_PRESET"
            self.state_started_at = time.monotonic()
            return

        self.active = False
        self.state = "DONE"
        self.get_logger().warn("Gimbal scan FSM completed.")

    def current_preset_name(self):
        return self.scan_sequence[self.sequence_index]

    def pwm_for_preset(self, preset_name):
        if preset_name == "PRESET_FAR":
            return self.preset_far_pwm
        if preset_name == "PRESET_NEAR":
            return self.preset_near_pwm
        raise ValueError(f"Unsupported gimbal preset: {preset_name}")

    def parse_sequence(self, value):
        sequence = []
        for token in value.split(","):
            preset_name = token.strip().upper()
            if not preset_name:
                continue
            self.pwm_for_preset(preset_name)
            sequence.append(preset_name)
        return sequence


def main(args=None):
    rclpy.init(args=args)
    node = GimbalScanFsmNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
