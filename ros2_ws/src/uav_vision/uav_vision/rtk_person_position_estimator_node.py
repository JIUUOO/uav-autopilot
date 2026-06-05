#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from std_msgs.msg import Float64
from uav_interfaces.msg import PersonPositionEstimate
from uav_interfaces.msg import TargetFeedback


EARTH_RADIUS_M = 6378137.0


class RtkPersonPositionEstimatorNode(Node):
    """
    Estimate a person lat/lon from RTK UAV position, heading, and target feedback.

    This is an empirical estimator: distance comes from the VLM distance bucket and
    lateral offset comes from bbox horizontal error. It is not camera-calibrated 3D projection.
    """

    def __init__(self):
        super().__init__("rtk_person_position_estimator")

        self.declare_parameter("target_feedback_topic", "/uav/vision/target_feedback")
        self.declare_parameter("gps_topic", "/mavros/global_position/global")
        self.declare_parameter("heading_topic", "/mavros/global_position/compass_hdg")
        self.declare_parameter("estimate_topic", "/uav/vision/person_position_estimate")
        self.declare_parameter("feedback_timeout_sec", 5.0)
        self.declare_parameter("gps_timeout_sec", 3.0)
        self.declare_parameter("heading_timeout_sec", 3.0)
        self.declare_parameter("min_priority_score", 0.50)
        self.declare_parameter("require_ready_to_inspect_for_estimate", False)
        self.declare_parameter("near_distance_m", 2.0)
        self.declare_parameter("far_distance_m", 5.0)
        self.declare_parameter("unknown_distance_m", 3.0)
        self.declare_parameter("lateral_error_gain_m", 4.0)
        self.declare_parameter("max_lateral_offset_m", 2.0)
        self.declare_parameter("horizontal_error_margin_m", 2.0)
        self.declare_parameter("assumed_target_below_m", 0.0)

        self.target_feedback_topic = str(self.get_parameter("target_feedback_topic").value)
        self.gps_topic = str(self.get_parameter("gps_topic").value)
        self.heading_topic = str(self.get_parameter("heading_topic").value)
        self.estimate_topic = str(self.get_parameter("estimate_topic").value)
        self.feedback_timeout_sec = float(self.get_parameter("feedback_timeout_sec").value)
        self.gps_timeout_sec = float(self.get_parameter("gps_timeout_sec").value)
        self.heading_timeout_sec = float(self.get_parameter("heading_timeout_sec").value)
        self.min_priority_score = float(self.get_parameter("min_priority_score").value)
        self.require_ready_to_inspect_for_estimate = bool(
            self.get_parameter("require_ready_to_inspect_for_estimate").value
        )
        self.near_distance_m = float(self.get_parameter("near_distance_m").value)
        self.far_distance_m = float(self.get_parameter("far_distance_m").value)
        self.unknown_distance_m = float(self.get_parameter("unknown_distance_m").value)
        self.lateral_error_gain_m = float(self.get_parameter("lateral_error_gain_m").value)
        self.max_lateral_offset_m = float(self.get_parameter("max_lateral_offset_m").value)
        self.horizontal_error_margin_m = float(self.get_parameter("horizontal_error_margin_m").value)
        self.assumed_target_below_m = float(self.get_parameter("assumed_target_below_m").value)

        self.latest_gps = None
        self.latest_gps_time = 0.0
        self.latest_heading_deg = None
        self.latest_heading_time = 0.0

        self.create_subscription(NavSatFix, self.gps_topic, self.on_gps, 10)
        self.create_subscription(Float64, self.heading_topic, self.on_heading, 10)
        self.create_subscription(TargetFeedback, self.target_feedback_topic, self.on_feedback, 10)
        self.estimate_pub = self.create_publisher(PersonPositionEstimate, self.estimate_topic, 10)

        self.get_logger().warn(
            f"RTK person position estimator started: feedback={self.target_feedback_topic}, "
            f"gps={self.gps_topic}, heading={self.heading_topic}, estimate={self.estimate_topic}"
        )

    def on_gps(self, msg):
        self.latest_gps = msg
        self.latest_gps_time = time.monotonic()

    def on_heading(self, msg):
        self.latest_heading_deg = float(msg.data) % 360.0
        self.latest_heading_time = time.monotonic()

    def on_feedback(self, feedback):
        now = time.monotonic()
        estimate = self.make_estimate(feedback, now)
        self.estimate_pub.publish(estimate)

    def make_estimate(self, feedback, now):
        msg = PersonPositionEstimate()
        msg.header = feedback.header
        msg.estimate_source = "rtk_feedback_bucket_v1"
        msg.track_id = int(feedback.track_id)
        msg.confidence = float(feedback.priority_score)
        msg.distance_bucket = feedback.distance_bucket or "unknown"

        if not self.feedback_usable(feedback):
            msg.has_estimate = False
            msg.reason = self.feedback_reject_reason(feedback)
            return msg

        if not self.gps_fresh(now):
            msg.has_estimate = False
            msg.reason = "gps unavailable or stale"
            return msg

        if not self.heading_fresh(now):
            msg.has_estimate = False
            msg.reason = "heading unavailable or stale"
            return msg

        distance_m = self.distance_from_bucket(msg.distance_bucket)
        body_right_m = self.clamp(
            float(feedback.error_x_norm) * self.lateral_error_gain_m,
            -self.max_lateral_offset_m,
            self.max_lateral_offset_m,
        )
        body_forward_m = distance_m
        north_m, east_m = self.body_to_ne_offset(
            body_forward_m,
            body_right_m,
            self.latest_heading_deg,
        )
        est_lat, est_lon = self.offset_lat_lon(
            self.latest_gps.latitude,
            self.latest_gps.longitude,
            north_m,
            east_m,
        )

        msg.has_estimate = True
        msg.reason = "estimated from RTK UAV position, compass heading, and empirical feedback distance"
        msg.uav_latitude_deg = float(self.latest_gps.latitude)
        msg.uav_longitude_deg = float(self.latest_gps.longitude)
        msg.uav_altitude_m = float(self.latest_gps.altitude)
        msg.heading_deg = float(self.latest_heading_deg)
        msg.target_body_forward_m = float(body_forward_m)
        msg.target_body_right_m = float(body_right_m)
        msg.target_distance_m = float(math.hypot(body_forward_m, body_right_m))
        msg.estimated_horizontal_error_m = self.horizontal_error_margin_m
        msg.estimated_latitude_deg = float(est_lat)
        msg.estimated_longitude_deg = float(est_lon)
        msg.estimated_altitude_m = float(self.latest_gps.altitude - self.assumed_target_below_m)
        return msg

    def feedback_usable(self, feedback):
        if not feedback.has_target:
            return False
        if feedback.priority_score < self.min_priority_score:
            return False
        if self.require_ready_to_inspect_for_estimate and not feedback.ready_to_inspect:
            return False
        return True

    def feedback_reject_reason(self, feedback):
        if not feedback.has_target:
            return "target feedback has no target"
        if feedback.priority_score < self.min_priority_score:
            return "target priority below threshold"
        if self.require_ready_to_inspect_for_estimate and not feedback.ready_to_inspect:
            return "target is not ready_to_inspect"
        return "target feedback rejected"

    def gps_fresh(self, now):
        if self.latest_gps is None:
            return False
        if now - self.latest_gps_time > self.gps_timeout_sec:
            return False
        return self.latest_gps.status.status != NavSatStatus.STATUS_NO_FIX

    def heading_fresh(self, now):
        return (
            self.latest_heading_deg is not None
            and now - self.latest_heading_time <= self.heading_timeout_sec
        )

    def distance_from_bucket(self, bucket):
        bucket = (bucket or "unknown").lower()
        if bucket == "near":
            return self.near_distance_m
        if bucket == "far":
            return self.far_distance_m
        return self.unknown_distance_m

    @staticmethod
    def body_to_ne_offset(forward_m, right_m, heading_deg):
        heading_rad = math.radians(heading_deg)
        north_m = forward_m * math.cos(heading_rad) - right_m * math.sin(heading_rad)
        east_m = forward_m * math.sin(heading_rad) + right_m * math.cos(heading_rad)
        return north_m, east_m

    @staticmethod
    def offset_lat_lon(latitude_deg, longitude_deg, north_m, east_m):
        latitude_rad = math.radians(latitude_deg)
        new_latitude_deg = latitude_deg + math.degrees(north_m / EARTH_RADIUS_M)
        longitude_scale = max(math.cos(latitude_rad), 1.0e-6)
        new_longitude_deg = longitude_deg + math.degrees(east_m / (EARTH_RADIUS_M * longitude_scale))
        return new_latitude_deg, new_longitude_deg

    @staticmethod
    def clamp(value, min_value, max_value):
        return max(min(value, max_value), min_value)


def main(args=None):
    rclpy.init(args=args)
    node = RtkPersonPositionEstimatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
