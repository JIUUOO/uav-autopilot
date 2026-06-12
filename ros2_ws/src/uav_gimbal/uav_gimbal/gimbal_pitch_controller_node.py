#!/usr/bin/env python3

import time

import rclpy
from mavros_msgs.srv import CommandLong
from rclpy.node import Node
from std_msgs.msg import UInt16
from uav_interfaces.msg import GimbalState


MAV_CMD_DO_SET_SERVO = 183


class GimbalPitchControllerNode(Node):
    """
    - Convert direct PWM commands into a safe PWM target.
    - Apply clamp, rate limit, and timeout before any actuator command.
    - Send Pixhawk servo PWM through MAVROS CommandLong only when dry_run is false.
    """

    def __init__(self):
        super().__init__("gimbal_pitch_controller")

        self.declare_parameter("dry_run", True)
        self.declare_parameter("command_service", "/mavros/cmd/command")
        self.declare_parameter("pitch_target_topic", "/uav/gimbal/pitch_target_pwm")
        self.declare_parameter("state_topic", "/uav/gimbal/state")

        self.declare_parameter("servo_channel", 7)  # Pitch gimbal is wired to Pixhawk PWM OUT 7.
        self.declare_parameter("pwm_min", 1550)
        self.declare_parameter("pwm_neutral", 1550)
        self.declare_parameter("pwm_max", 1690)
        self.declare_parameter("invert_pwm", False)

        self.declare_parameter("command_hz", 5.0)
        self.declare_parameter("max_rate_pwm_s", 100.0)
        self.declare_parameter("command_timeout_sec", 10.0)
        self.declare_parameter("timeout_to_neutral", True)

        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.command_service = str(self.get_parameter("command_service").value)
        self.pitch_target_topic = str(self.get_parameter("pitch_target_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)

        self.servo_channel = int(self.get_parameter("servo_channel").value)
        self.pwm_min = int(self.get_parameter("pwm_min").value)
        self.pwm_neutral = int(self.get_parameter("pwm_neutral").value)
        self.pwm_max = int(self.get_parameter("pwm_max").value)
        self.invert_pwm = bool(self.get_parameter("invert_pwm").value)

        self.command_hz = float(self.get_parameter("command_hz").value)
        self.max_rate_pwm_s = float(self.get_parameter("max_rate_pwm_s").value)
        self.command_timeout_sec = float(self.get_parameter("command_timeout_sec").value)
        self.timeout_to_neutral = bool(self.get_parameter("timeout_to_neutral").value)

        self.pitch_target_pwm = self.clamp_pwm(self.pwm_neutral)
        self.pitch_command_pwm = self.pitch_target_pwm
        self.last_command_time = None
        self.last_timer_time = time.monotonic()
        self.last_source = "init"
        self.last_safety_state = "dry_run" if self.dry_run else "waiting_service"

        self.command_client = self.create_client(CommandLong, self.command_service)
        self.create_subscription(UInt16, self.pitch_target_topic, self.on_pitch_target, 10)
        self.state_pub = self.create_publisher(GimbalState, self.state_topic, 10)

        timer_period = 1.0 / max(self.command_hz, 0.1)
        self.timer = self.create_timer(timer_period, self.on_timer)

        self.get_logger().warn(
            f"Gimbal pitch controller started: dry_run={self.dry_run}, "
            f"servo_channel={self.servo_channel}, "
            f"pwm_min={self.pwm_min}, pwm_neutral={self.pwm_neutral}, "
            f"pwm_max={self.pwm_max}, invert_pwm={self.invert_pwm}, "
            f"service={self.command_service}"
        )

    def on_pitch_target(self, msg):
        self.pitch_target_pwm = self.clamp_pwm(float(msg.data))
        self.last_command_time = time.monotonic()
        self.last_source = "pitch_target_topic"
        self.last_safety_state = "active"

    def on_timer(self):
        now = time.monotonic()
        dt = max(now - self.last_timer_time, 0.0)
        self.last_timer_time = now

        command_active = self.is_command_active(now)
        desired_command_pwm = self.pitch_target_pwm

        if not command_active and self.timeout_to_neutral:
            desired_command_pwm = self.pwm_neutral
            self.last_safety_state = "timeout_neutral"
        elif not command_active:
            self.last_safety_state = "timeout_hold"

        desired_output_pwm = self.command_to_output_pwm(desired_command_pwm)
        self.pitch_command_pwm = self.rate_limited_pwm(
            current=self.pitch_command_pwm,
            target=desired_output_pwm,
            dt=dt,
        )

        pwm = self.clamp_pwm(self.pitch_command_pwm)
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
        state.pitch_target_pwm = int(self.pitch_target_pwm)
        state.pitch_command_pwm = int(self.pitch_command_pwm)
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

    def rate_limited_pwm(self, *, current: int, target: int, dt: float) -> int:
        max_delta = max(self.max_rate_pwm_s * dt, 0.0)
        delta = target - current
        if abs(delta) <= max_delta:
            return target
        if delta > 0:
            return self.clamp_pwm(current + max_delta)
        return self.clamp_pwm(current - max_delta)

    def clamp_pwm(self, value: float) -> int:
        return int(round(max(self.pwm_min, min(self.pwm_max, value))))

    def command_to_output_pwm(self, value: float) -> int:
        command_pwm = self.clamp_pwm(value)
        if not self.invert_pwm:
            return command_pwm

        return self.clamp_pwm(self.pwm_neutral - (command_pwm - self.pwm_neutral))


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
