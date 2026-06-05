#!/usr/bin/env python3

import time

import rclpy
from mavros_msgs.srv import CommandLong
from rclpy.node import Node
from std_msgs.msg import Float32
from uav_interfaces.msg import GeminiReport
from uav_interfaces.msg import GimbalState


MAV_CMD_DO_SET_SERVO = 183


class GimbalPitchControllerNode(Node):
    """
    - Convert direct pitch intent into a safe pitch target.
    - Apply clamp, rate limit, and timeout before any actuator command.
    - Send Pixhawk servo PWM through MAVROS CommandLong only when dry_run is false.
    """

    def __init__(self):
        super().__init__("gimbal_pitch_controller")

        self.declare_parameter("dry_run", True)
        self.declare_parameter("command_service", "/mavros/cmd/command")
        self.declare_parameter("gemini_report_topic", "/uav/vision/gemini_report")
        self.declare_parameter("pitch_target_topic", "/uav/gimbal/pitch_target_deg")
        self.declare_parameter("state_topic", "/uav/gimbal/state")

        self.declare_parameter("servo_channel", 7)  # Pitch gimbal is wired to Pixhawk PWM OUT 7.
        self.declare_parameter("pitch_min_deg", -45.0)
        self.declare_parameter("pitch_max_deg", 20.0)
        self.declare_parameter("pitch_neutral_deg", 0.0)
        self.declare_parameter("pwm_min", 1200)
        self.declare_parameter("pwm_center", 1500)
        self.declare_parameter("pwm_max", 1800)
        self.declare_parameter("invert_pitch_pwm", True)

        self.declare_parameter("command_hz", 5.0)
        self.declare_parameter("max_rate_deg_s", 20.0)
        self.declare_parameter("command_timeout_sec", 2.0)
        self.declare_parameter("timeout_to_neutral", True)

        self.declare_parameter("confidence_threshold", 0.75)
        self.declare_parameter("preset_far_deg", 10.0)
        self.declare_parameter("preset_near_deg", 20.0)

        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.command_service = str(self.get_parameter("command_service").value)
        self.gemini_report_topic = str(self.get_parameter("gemini_report_topic").value)
        self.pitch_target_topic = str(self.get_parameter("pitch_target_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)

        self.servo_channel = int(self.get_parameter("servo_channel").value)
        self.pitch_min_deg = float(self.get_parameter("pitch_min_deg").value)
        self.pitch_max_deg = float(self.get_parameter("pitch_max_deg").value)
        self.pitch_neutral_deg = float(self.get_parameter("pitch_neutral_deg").value)
        self.pwm_min = int(self.get_parameter("pwm_min").value)
        self.pwm_center = int(self.get_parameter("pwm_center").value)
        self.pwm_max = int(self.get_parameter("pwm_max").value)
        self.invert_pitch_pwm = bool(self.get_parameter("invert_pitch_pwm").value)

        self.command_hz = float(self.get_parameter("command_hz").value)
        self.max_rate_deg_s = float(self.get_parameter("max_rate_deg_s").value)
        self.command_timeout_sec = float(self.get_parameter("command_timeout_sec").value)
        self.timeout_to_neutral = bool(self.get_parameter("timeout_to_neutral").value)

        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.preset_far_deg = float(self.get_parameter("preset_far_deg").value)
        self.preset_near_deg = float(self.get_parameter("preset_near_deg").value)

        self.pitch_target_deg = self.clamp_pitch(self.pitch_neutral_deg)
        self.pitch_command_deg = self.pitch_target_deg
        self.last_command_time = None
        self.last_timer_time = time.monotonic()
        self.last_source = "init"
        self.last_safety_state = "dry_run" if self.dry_run else "waiting_service"

        self.command_client = self.create_client(CommandLong, self.command_service)
        self.create_subscription(GeminiReport, self.gemini_report_topic, self.on_gemini_report, 10)
        self.create_subscription(Float32, self.pitch_target_topic, self.on_pitch_target, 10)
        self.state_pub = self.create_publisher(GimbalState, self.state_topic, 10)

        timer_period = 1.0 / max(self.command_hz, 0.1)
        self.timer = self.create_timer(timer_period, self.on_timer)

        self.get_logger().warn(
            f"Gimbal pitch controller started: dry_run={self.dry_run}, "
            f"servo_channel={self.servo_channel}, invert_pitch_pwm={self.invert_pitch_pwm}, "
            f"service={self.command_service}"
        )

    def on_pitch_target(self, msg):
        self.pitch_target_deg = self.clamp_pitch(float(msg.data))
        self.last_command_time = time.monotonic()
        self.last_source = "pitch_target_topic"
        self.last_safety_state = "active"

    def on_gemini_report(self, msg):
        if not msg.parsed_ok or not msg.person_detected:
            return

        candidate = self.find_candidate(msg.person_candidates, msg.primary_candidate_index)
        if candidate is None:
            self.last_safety_state = "missing_primary_candidate"
            return
        if candidate.confidence < self.confidence_threshold:
            self.last_safety_state = "low_confidence"
            return

        preset = msg.recommended_gimbal_preset.strip().upper()
        if preset == "PRESET_FAR":
            self.pitch_target_deg = self.clamp_pitch(self.preset_far_deg)
        elif preset == "PRESET_NEAR":
            self.pitch_target_deg = self.clamp_pitch(self.preset_near_deg)
        elif preset == "HOLD":
            return
        else:
            self.last_safety_state = "unknown_gimbal_preset"
            return

        self.last_command_time = time.monotonic()
        self.last_source = f"gemini:{preset.lower()}"
        self.last_safety_state = "active"

    def on_timer(self):
        now = time.monotonic()
        dt = max(now - self.last_timer_time, 0.0)
        self.last_timer_time = now

        command_active = self.is_command_active(now)
        desired_pitch = self.pitch_target_deg

        if not command_active and self.timeout_to_neutral:
            desired_pitch = self.pitch_neutral_deg
            self.last_safety_state = "timeout_neutral"
        elif not command_active:
            self.last_safety_state = "timeout_hold"

        self.pitch_command_deg = self.rate_limited_pitch(
            current=self.pitch_command_deg,
            target=self.clamp_pitch(desired_pitch),
            dt=dt,
        )

        pwm = self.pitch_to_pwm(self.pitch_command_deg)
        if not self.dry_run:
            self.send_servo_pwm(pwm)

        self.publish_state(pwm=pwm, command_active=command_active)

    def send_servo_pwm(self, pwm: int):
        if not self.command_client.service_is_ready():
            self.last_safety_state = "service_unavailable"
            return

        request = CommandLong.Request()
        request.broadcast = False
        request.command = MAV_CMD_DO_SET_SERVO
        request.confirmation = 0
        request.param1 = float(self.servo_channel)
        request.param2 = float(pwm)
        request.param3 = 0.0
        request.param4 = 0.0
        request.param5 = 0.0
        request.param6 = 0.0
        request.param7 = 0.0

        future = self.command_client.call_async(request)
        future.add_done_callback(self.on_command_result)

    def on_command_result(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.last_safety_state = "service_error"
            self.get_logger().error(f"Gimbal PWM command service failed: {exc}")
            return

        if not result.success:
            self.last_safety_state = f"command_rejected:{result.result}"

    def publish_state(self, *, pwm: int, command_active: bool):
        state = GimbalState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.pitch_target_deg = float(self.pitch_target_deg)
        state.pitch_command_deg = float(self.pitch_command_deg)
        state.pitch_pwm = int(pwm)
        state.command_active = bool(command_active)
        state.dry_run = bool(self.dry_run)
        state.mode = "dry_run" if self.dry_run else "pwm"
        state.safety_state = self.last_safety_state
        state.source = self.last_source
        self.state_pub.publish(state)

    def is_command_active(self, now: float) -> bool:
        if self.last_command_time is None:
            return False
        return (now - self.last_command_time) <= self.command_timeout_sec

    def rate_limited_pitch(self, *, current: float, target: float, dt: float) -> float:
        max_delta = max(self.max_rate_deg_s * dt, 0.0)
        delta = target - current
        if abs(delta) <= max_delta:
            return target
        if delta > 0:
            return current + max_delta
        return current - max_delta

    def pitch_to_pwm(self, pitch_deg: float) -> int:
        pitch_deg = self.clamp_pitch(pitch_deg)

        if pitch_deg >= self.pitch_neutral_deg:
            span_deg = max(self.pitch_max_deg - self.pitch_neutral_deg, 0.001)
            ratio = (pitch_deg - self.pitch_neutral_deg) / span_deg
            if self.invert_pitch_pwm:
                pwm = self.pwm_center - ratio * (self.pwm_center - self.pwm_min)
            else:
                pwm = self.pwm_center + ratio * (self.pwm_max - self.pwm_center)
        else:
            span_deg = max(self.pitch_neutral_deg - self.pitch_min_deg, 0.001)
            ratio = (self.pitch_neutral_deg - pitch_deg) / span_deg
            if self.invert_pitch_pwm:
                pwm = self.pwm_center + ratio * (self.pwm_max - self.pwm_center)
            else:
                pwm = self.pwm_center - ratio * (self.pwm_center - self.pwm_min)

        return int(round(max(min(pwm, self.pwm_max), self.pwm_min)))

    def clamp_pitch(self, value: float) -> float:
        return max(self.pitch_min_deg, min(self.pitch_max_deg, value))

    @staticmethod
    def find_candidate(candidates, candidate_index):
        return next(
            (
                candidate
                for candidate in candidates
                if candidate.candidate_index == candidate_index
            ),
            None,
        )


def main(args=None):
    rclpy.init(args=args)
    node = GimbalPitchControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
