#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from uav_interfaces.msg import CandidateTrackArray
from uav_interfaces.msg import TargetFeedback


class TargetFeedbackNode(Node):
    """
    Convert the selected image-space candidate track into deterministic motion intent.

    This node publishes body-frame movement recommendations only; it does not send
    MAVLink/Pixhawk commands or enforce mission safety boundaries.
    """

    def __init__(self):
        super().__init__("target_feedback")

        self.declare_parameter("tracks_topic", "/uav/vision/candidate_tracks")
        self.declare_parameter("feedback_topic", "/uav/vision/target_feedback")
        self.declare_parameter("min_priority_score", 0.50)
        self.declare_parameter("center_x_min", 0.40)
        self.declare_parameter("center_x_max", 0.60)
        self.declare_parameter("center_y_min", 0.35)
        self.declare_parameter("center_y_max", 0.70)
        self.declare_parameter("side_step_m", 0.30)
        self.declare_parameter("forward_step_m", 0.50)
        self.declare_parameter("approach_distance_bucket", "far")
        self.declare_parameter("inspect_distance_bucket", "near")
        self.declare_parameter("require_centered_for_inspect", True)

        self.tracks_topic = str(self.get_parameter("tracks_topic").value)
        self.feedback_topic = str(self.get_parameter("feedback_topic").value)
        self.min_priority_score = float(self.get_parameter("min_priority_score").value)
        self.center_x_min = float(self.get_parameter("center_x_min").value)
        self.center_x_max = float(self.get_parameter("center_x_max").value)
        self.center_y_min = float(self.get_parameter("center_y_min").value)
        self.center_y_max = float(self.get_parameter("center_y_max").value)
        self.side_step_m = float(self.get_parameter("side_step_m").value)
        self.forward_step_m = float(self.get_parameter("forward_step_m").value)
        self.approach_distance_bucket = str(self.get_parameter("approach_distance_bucket").value).lower()
        self.inspect_distance_bucket = str(self.get_parameter("inspect_distance_bucket").value).lower()
        self.require_centered_for_inspect = bool(self.get_parameter("require_centered_for_inspect").value)

        self.create_subscription(CandidateTrackArray, self.tracks_topic, self.on_tracks, 10)
        self.feedback_pub = self.create_publisher(TargetFeedback, self.feedback_topic, 10)

        self.get_logger().warn(
            f"Target feedback started: tracks={self.tracks_topic}, "
            f"feedback={self.feedback_topic}, min_priority={self.min_priority_score:.2f}"
        )

    def on_tracks(self, tracks_msg):
        track = self.select_track(tracks_msg)
        if track is None:
            self.publish_no_target(tracks_msg, reason="no candidate track over priority threshold")
            return

        feedback = self.make_feedback(tracks_msg, track)
        self.feedback_pub.publish(feedback)

    def select_track(self, tracks_msg):
        if not tracks_msg.tracks:
            return None

        if tracks_msg.primary_track_id:
            for track in tracks_msg.tracks:
                if track.track_id == tracks_msg.primary_track_id:
                    if track.priority_score >= self.min_priority_score:
                        return track
                    return None

        for track in tracks_msg.tracks:
            if track.priority_score >= self.min_priority_score:
                return track
        return None

    def make_feedback(self, tracks_msg, track):
        feedback = TargetFeedback()
        feedback.header = tracks_msg.header
        feedback.has_target = True
        feedback.track_id = int(track.track_id)
        feedback.priority_score = float(track.priority_score)
        feedback.center_x_norm = float(track.center_x_norm)
        feedback.center_y_norm = float(track.center_y_norm)
        feedback.bbox_area_norm = float(track.bbox_area_norm)
        feedback.distance_bucket = track.distance_bucket or "unknown"

        target_x = (self.center_x_min + self.center_x_max) / 2.0
        target_y = (self.center_y_min + self.center_y_max) / 2.0
        feedback.error_x_norm = feedback.center_x_norm - target_x
        feedback.error_y_norm = feedback.center_y_norm - target_y

        feedback.horizontal_command = self.horizontal_command(feedback.center_x_norm)
        feedback.vertical_command = self.vertical_command(feedback.center_y_norm)
        feedback.range_command = self.range_command(feedback.distance_bucket)
        feedback.centered = (
            feedback.horizontal_command == "CENTER"
            and feedback.vertical_command == "CENTER"
        )
        feedback.ready_to_inspect = self.ready_to_inspect(feedback)
        feedback.motion_command = self.motion_command(feedback)
        feedback.recommended_body_forward_m = self.recommended_forward_m(feedback)
        feedback.recommended_body_right_m = self.recommended_right_m(feedback)
        feedback.reason = self.reason(feedback)
        return feedback

    def publish_no_target(self, tracks_msg, reason):
        feedback = TargetFeedback()
        feedback.header = tracks_msg.header
        feedback.has_target = False
        feedback.track_id = 0
        feedback.distance_bucket = "unknown"
        feedback.horizontal_command = "NONE"
        feedback.vertical_command = "NONE"
        feedback.range_command = "NONE"
        feedback.motion_command = "SEARCH"
        feedback.reason = reason
        self.feedback_pub.publish(feedback)

    def horizontal_command(self, center_x):
        if center_x < self.center_x_min:
            return "LEFT"
        if center_x > self.center_x_max:
            return "RIGHT"
        return "CENTER"

    def vertical_command(self, center_y):
        if center_y < self.center_y_min:
            return "UPPER"
        if center_y > self.center_y_max:
            return "LOWER"
        return "CENTER"

    def range_command(self, distance_bucket):
        bucket = (distance_bucket or "unknown").lower()
        if bucket == self.approach_distance_bucket:
            return "APPROACH"
        if bucket == self.inspect_distance_bucket:
            return "HOLD"
        return "UNKNOWN"

    def ready_to_inspect(self, feedback):
        if feedback.range_command != "HOLD":
            return False
        if self.require_centered_for_inspect and not feedback.centered:
            return False
        return True

    def motion_command(self, feedback):
        if feedback.ready_to_inspect:
            return "INSPECT"
        if feedback.horizontal_command != "CENTER":
            return f"STRAFE_{feedback.horizontal_command}"
        if feedback.range_command == "APPROACH":
            return "APPROACH"
        return "HOLD"

    def recommended_forward_m(self, feedback):
        if feedback.range_command == "APPROACH" and feedback.horizontal_command == "CENTER":
            return self.forward_step_m
        return 0.0

    def recommended_right_m(self, feedback):
        if feedback.horizontal_command == "LEFT":
            return -abs(self.side_step_m)
        if feedback.horizontal_command == "RIGHT":
            return abs(self.side_step_m)
        return 0.0

    def reason(self, feedback):
        if feedback.motion_command == "INSPECT":
            return "target centered and close enough for inspection"
        if feedback.motion_command.startswith("STRAFE"):
            return "target is horizontally off-center in the image"
        if feedback.motion_command == "APPROACH":
            return "target is centered but visually far"
        return "target not actionable or already held"


def main(args=None):
    rclpy.init(args=args)
    node = TargetFeedbackNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
