#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from uav_interfaces.msg import TopdownCenteringFeedback
from uav_interfaces.msg import TrackedTarget


class TopdownCenteringFeedbackNode(Node):
    """
    Convert a top-down tracked target's image error into body-frame XY movement intent.

    This node only publishes bounded movement recommendations. It does not command
    Pixhawk, change flight modes, or decide whether vehicle movement is safe.
    """

    def __init__(self):
        super().__init__("topdown_centering_feedback")

        self.declare_parameter("tracked_target_topic", "/uav/vision/tracked_target")
        self.declare_parameter(
            "feedback_topic",
            "/uav/vision/topdown_centering_feedback",
        )
        self.declare_parameter("target_center_x_norm", 0.50)
        self.declare_parameter("target_center_y_norm", 0.50)
        self.declare_parameter("center_tolerance_x_norm", 0.06)
        self.declare_parameter("center_tolerance_y_norm", 0.06)
        self.declare_parameter("body_forward_gain_m", 2.0)
        self.declare_parameter("body_right_gain_m", 2.0)
        self.declare_parameter("body_forward_sign", -1.0)
        self.declare_parameter("body_right_sign", 1.0)
        self.declare_parameter("swap_image_axes", False)
        self.declare_parameter("max_step_m", 0.30)
        self.declare_parameter("min_step_m", 0.05)
        self.declare_parameter("required_centered_frames", 10)
        self.declare_parameter("required_centered_sec", 1.0)

        self.tracked_target_topic = str(
            self.get_parameter("tracked_target_topic").value
        )
        self.feedback_topic = str(self.get_parameter("feedback_topic").value)
        self.target_center_x_norm = float(
            self.get_parameter("target_center_x_norm").value
        )
        self.target_center_y_norm = float(
            self.get_parameter("target_center_y_norm").value
        )
        self.center_tolerance_x_norm = max(
            float(self.get_parameter("center_tolerance_x_norm").value),
            0.0,
        )
        self.center_tolerance_y_norm = max(
            float(self.get_parameter("center_tolerance_y_norm").value),
            0.0,
        )
        self.body_forward_gain_m = abs(
            float(self.get_parameter("body_forward_gain_m").value)
        )
        self.body_right_gain_m = abs(
            float(self.get_parameter("body_right_gain_m").value)
        )
        self.body_forward_sign = self.normalized_sign(
            self.get_parameter("body_forward_sign").value
        )
        self.body_right_sign = self.normalized_sign(
            self.get_parameter("body_right_sign").value
        )
        self.swap_image_axes = bool(self.get_parameter("swap_image_axes").value)
        self.max_step_m = max(float(self.get_parameter("max_step_m").value), 0.0)
        self.min_step_m = min(
            max(float(self.get_parameter("min_step_m").value), 0.0),
            self.max_step_m,
        )
        self.required_centered_frames = max(
            int(self.get_parameter("required_centered_frames").value),
            1,
        )
        self.required_centered_sec = max(
            float(self.get_parameter("required_centered_sec").value),
            0.0,
        )

        self.centered_frames = 0
        self.centered_since_sec = None

        self.create_subscription(
            TrackedTarget,
            self.tracked_target_topic,
            self.on_tracked_target,
            10,
        )
        self.feedback_pub = self.create_publisher(
            TopdownCenteringFeedback,
            self.feedback_topic,
            10,
        )

        self.get_logger().warn(
            f"Top-down centering feedback started: tracked={self.tracked_target_topic}, "
            f"output={self.feedback_topic}, max_step={self.max_step_m:.2f}m, "
            f"required_centered_frames={self.required_centered_frames}, "
            f"required_centered_sec={self.required_centered_sec:.2f}s, "
            f"swap_image_axes={self.swap_image_axes}, "
            f"body_forward_sign={self.body_forward_sign:+.0f}, "
            f"body_right_sign={self.body_right_sign:+.0f}"
        )

    def on_tracked_target(self, tracked):
        if not tracked.tracking:
            self.reset_centered_state()
            self.publish_no_target(tracked)
            return

        error_x = float(tracked.center_x_norm) - self.target_center_x_norm
        error_y = float(tracked.center_y_norm) - self.target_center_y_norm
        centered = (
            abs(error_x) <= self.center_tolerance_x_norm
            and abs(error_y) <= self.center_tolerance_y_norm
        )
        stamp_sec = self.stamp_sec(tracked.header.stamp)
        if centered:
            if self.centered_since_sec is None or stamp_sec < self.centered_since_sec:
                self.centered_since_sec = stamp_sec
                self.centered_frames = 0
            self.centered_frames += 1
        else:
            self.reset_centered_state()
        centered_duration_sec = (
            max(stamp_sec - self.centered_since_sec, 0.0)
            if self.centered_since_sec is not None
            else 0.0
        )

        feedback = TopdownCenteringFeedback()
        feedback.header = tracked.header
        feedback.has_target = True
        feedback.target_label = tracked.target_label
        feedback.status = "centered" if centered else "correction_required"
        feedback.center_x_norm = float(tracked.center_x_norm)
        feedback.center_y_norm = float(tracked.center_y_norm)
        feedback.error_x_norm = error_x
        feedback.error_y_norm = error_y
        forward_error = error_x if self.swap_image_axes else error_y
        right_error = error_y if self.swap_image_axes else error_x
        forward_tolerance = (
            self.center_tolerance_x_norm
            if self.swap_image_axes
            else self.center_tolerance_y_norm
        )
        right_tolerance = (
            self.center_tolerance_y_norm
            if self.swap_image_axes
            else self.center_tolerance_x_norm
        )
        feedback.recommended_body_forward_m = self.axis_recommendation(
            error=forward_error,
            tolerance=forward_tolerance,
            gain=self.body_forward_gain_m,
            sign=self.body_forward_sign,
        )
        feedback.recommended_body_right_m = self.axis_recommendation(
            error=right_error,
            tolerance=right_tolerance,
            gain=self.body_right_gain_m,
            sign=self.body_right_sign,
        )
        feedback.centered = centered
        feedback.centered_frames = self.centered_frames
        feedback.centered_duration_sec = centered_duration_sec
        feedback.ready_to_localize = (
            self.centered_frames >= self.required_centered_frames
            and centered_duration_sec >= self.required_centered_sec
        )
        if feedback.ready_to_localize:
            feedback.status = "ready_to_localize"

        self.feedback_pub.publish(feedback)

    def publish_no_target(self, tracked):
        feedback = TopdownCenteringFeedback()
        feedback.header = tracked.header
        feedback.has_target = False
        feedback.target_label = tracked.target_label
        feedback.status = tracked.status or "tracking_unavailable"
        self.feedback_pub.publish(feedback)

    def axis_recommendation(self, *, error, tolerance, gain, sign):
        if abs(error) <= tolerance:
            return 0.0

        recommendation = error * gain * sign
        recommendation = max(-self.max_step_m, min(self.max_step_m, recommendation))
        if 0.0 < abs(recommendation) < self.min_step_m:
            recommendation = self.min_step_m if recommendation > 0.0 else -self.min_step_m
        return float(recommendation)

    def reset_centered_state(self):
        self.centered_frames = 0
        self.centered_since_sec = None

    @staticmethod
    def normalized_sign(value):
        return -1.0 if float(value) < 0.0 else 1.0

    @staticmethod
    def stamp_sec(stamp):
        return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def main(args=None):
    rclpy.init(args=args)
    node = TopdownCenteringFeedbackNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
