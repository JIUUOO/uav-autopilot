#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from std_msgs.msg import UInt16


class GimbalScanFsmNode(Node):
    """
    - Rule-based fixed scan; Gemini does not adapt the order or number of presets.
    - Publish calibrated gimbal PWM presets in a deterministic scan sequence.
    - Wait for mechanical settle time after each preset.
    - Trigger Gemini analysis once the camera viewpoint should be stable.
    """

    def __init__(self):
        super().__init__("gimbal_scan_fsm")

        self.declare_parameter("pitch_target_topic", "/uav/gimbal/pitch_target_pwm")
        self.declare_parameter("trigger_topic", "/uav/vision/analyze_trigger")
        self.declare_parameter("preset_far_pwm", 1450)
        self.declare_parameter("preset_near_pwm", 1400)
        self.declare_parameter("scan_sequence", "PRESET_FAR,PRESET_NEAR")
        self.declare_parameter("settle_sec", 1.0)
        self.declare_parameter("post_trigger_sec", 0.2)
        self.declare_parameter("scan_count", 1)  # How many times to repeat the full gimbal scan sequence.
        self.declare_parameter("auto_start", True)
        self.declare_parameter("timer_hz", 10.0)

        self.pitch_target_topic = str(self.get_parameter("pitch_target_topic").value)
        self.trigger_topic = str(self.get_parameter("trigger_topic").value)
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
        self.scan_count = max(int(self.get_parameter("scan_count").value), 0)
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.timer_hz = max(float(self.get_parameter("timer_hz").value), 0.1)

        self.pitch_pub = self.create_publisher(UInt16, self.pitch_target_topic, 10)
        self.trigger_pub = self.create_publisher(Empty, self.trigger_topic, 10)

        self.active = False
        self.state = "IDLE"
        self.sequence_index = 0
        self.completed_scans = 0
        self.state_started_at = time.monotonic()

        self.timer = self.create_timer(1.0 / self.timer_hz, self.on_timer)

        if self.auto_start:
            self.start_scan()

        self.get_logger().warn(
            "Gimbal scan FSM started: "
            f"sequence={self.scan_sequence}, scan_count={self.scan_count}, "
            f"settle_sec={self.settle_sec}, trigger={self.trigger_topic}"
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
                self.trigger_pub.publish(Empty())
                self.get_logger().warn(
                    f"Gemini trigger published after {self.current_preset_name()}."
                )
                self.state = "POST_TRIGGER"
                self.state_started_at = now
            return

        if self.state == "POST_TRIGGER":
            if now - self.state_started_at >= self.post_trigger_sec:
                self.advance_sequence()

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
