#!/usr/bin/env python3

import math
import statistics
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from uav_interfaces.msg import TargetPositionEstimate
from uav_interfaces.msg import TopdownCenteringFeedback


EARTH_RADIUS_M = 6378137.0


class TopdownRtkLocalizerNode(Node):
    """
    Fix a target lat/lon from averaged RTK UAV positions after top-down centering.

    The empirical localization assumption is that the camera optical axis and the
    target coincide on the ground when ready_to_localize becomes true.
    """

    def __init__(self):
        super().__init__("topdown_rtk_localizer")

        self.declare_parameter(
            "centering_feedback_topic",
            "/uav/vision/topdown_centering_feedback",
        )
        self.declare_parameter("gps_topic", "/mavros/global_position/global")
        self.declare_parameter(
            "estimate_topic",
            "/uav/vision/target_position_estimate",
        )
        self.declare_parameter("sample_count", 5)
        self.declare_parameter("sampling_timeout_sec", 3.0)
        self.declare_parameter("max_horizontal_stddev_m", 1.0)
        self.declare_parameter("require_known_covariance", True)

        self.centering_feedback_topic = str(
            self.get_parameter("centering_feedback_topic").value
        )
        self.gps_topic = str(self.get_parameter("gps_topic").value)
        self.estimate_topic = str(self.get_parameter("estimate_topic").value)
        self.sample_count = max(int(self.get_parameter("sample_count").value), 1)
        self.sampling_timeout_sec = max(
            float(self.get_parameter("sampling_timeout_sec").value),
            0.1,
        )
        self.max_horizontal_stddev_m = max(
            float(self.get_parameter("max_horizontal_stddev_m").value),
            0.0,
        )
        self.require_known_covariance = bool(
            self.get_parameter("require_known_covariance").value
        )

        self.collecting = False
        self.ready_latched = False
        self.started_at = 0.0
        self.trigger_feedback = None
        self.samples = []

        self.create_subscription(
            TopdownCenteringFeedback,
            self.centering_feedback_topic,
            self.on_centering_feedback,
            10,
        )
        self.create_subscription(NavSatFix, self.gps_topic, self.on_gps, 10)
        self.estimate_pub = self.create_publisher(
            TargetPositionEstimate,
            self.estimate_topic,
            10,
        )
        self.timer = self.create_timer(0.1, self.on_timer)

        self.get_logger().warn(
            f"Top-down RTK localizer started: feedback={self.centering_feedback_topic}, "
            f"gps={self.gps_topic}, output={self.estimate_topic}, "
            f"samples={self.sample_count}, max_hstd={self.max_horizontal_stddev_m:.2f}m"
        )

    def on_centering_feedback(self, feedback):
        ready = bool(feedback.has_target and feedback.ready_to_localize)
        if not ready:
            self.ready_latched = False
            if self.collecting:
                self.publish_failure("top-down centering lost during RTK sampling")
                self.reset_collection()
            return

        if self.ready_latched or self.collecting:
            return

        self.ready_latched = True
        self.collecting = True
        self.started_at = time.monotonic()
        self.trigger_feedback = feedback
        self.samples = []
        self.get_logger().warn(
            f"RTK target localization sampling started: target={feedback.target_label!r}"
        )

    def on_gps(self, gps):
        if not self.collecting or not self.gps_usable(gps):
            return

        horizontal_stddev_m = self.horizontal_stddev_m(gps)
        if horizontal_stddev_m is None:
            if self.require_known_covariance:
                return
            horizontal_stddev_m = 0.0
        if horizontal_stddev_m > self.max_horizontal_stddev_m:
            return

        self.samples.append(
            (
                float(gps.latitude),
                float(gps.longitude),
                float(gps.altitude),
                horizontal_stddev_m,
                gps.header,
            )
        )
        if len(self.samples) >= self.sample_count:
            self.publish_estimate()
            self.reset_collection()

    def on_timer(self):
        if not self.collecting:
            return
        if time.monotonic() - self.started_at > self.sampling_timeout_sec:
            self.publish_failure(
                f"RTK sampling timeout: accepted {len(self.samples)}/{self.sample_count} samples"
            )
            self.reset_collection()

    def publish_estimate(self):
        latitudes = [sample[0] for sample in self.samples]
        longitudes = [sample[1] for sample in self.samples]
        altitudes = [sample[2] for sample in self.samples]
        reported_stddevs = [sample[3] for sample in self.samples]
        mean_latitude = statistics.fmean(latitudes)
        mean_longitude = statistics.fmean(longitudes)

        estimate = self.base_estimate()
        estimate.header = self.samples[-1][4]
        estimate.has_estimate = True
        estimate.reason = "target fixed from averaged RTK UAV position after stable top-down centering"
        estimate.sample_count = len(self.samples)
        estimate.estimated_horizontal_error_m = max(
            max(reported_stddevs),
            self.horizontal_sample_spread_m(
                latitudes,
                longitudes,
                mean_latitude,
                mean_longitude,
            ),
        )
        estimate.estimated_latitude_deg = mean_latitude
        estimate.estimated_longitude_deg = mean_longitude
        estimate.uav_altitude_m = statistics.fmean(altitudes)
        self.estimate_pub.publish(estimate)

        self.get_logger().warn(
            f"RTK target localized: lat={mean_latitude:.9f}, "
            f"lon={mean_longitude:.9f}, "
            f"error~{estimate.estimated_horizontal_error_m:.2f}m"
        )

    def publish_failure(self, reason):
        estimate = self.base_estimate()
        estimate.has_estimate = False
        estimate.reason = reason
        estimate.sample_count = len(self.samples)
        self.estimate_pub.publish(estimate)
        self.get_logger().error(reason)

    def base_estimate(self):
        estimate = TargetPositionEstimate()
        estimate.estimate_source = "topdown_rtk_average_v1"
        if self.trigger_feedback is not None:
            estimate.header = self.trigger_feedback.header
            estimate.target_label = self.trigger_feedback.target_label
            estimate.centered_error_x_norm = self.trigger_feedback.error_x_norm
            estimate.centered_error_y_norm = self.trigger_feedback.error_y_norm
        return estimate

    def reset_collection(self):
        self.collecting = False
        self.started_at = 0.0
        self.trigger_feedback = None
        self.samples = []

    @staticmethod
    def gps_usable(gps):
        return (
            gps.status.status != NavSatStatus.STATUS_NO_FIX
            and math.isfinite(gps.latitude)
            and math.isfinite(gps.longitude)
            and math.isfinite(gps.altitude)
        )

    @staticmethod
    def horizontal_stddev_m(gps):
        if gps.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            return None
        north_variance = max(float(gps.position_covariance[0]), 0.0)
        east_variance = max(float(gps.position_covariance[4]), 0.0)
        return math.sqrt(max(north_variance, east_variance))

    @staticmethod
    def horizontal_sample_spread_m(latitudes, longitudes, mean_latitude, mean_longitude):
        latitude_scale_m = math.pi * EARTH_RADIUS_M / 180.0
        longitude_scale_m = latitude_scale_m * max(
            math.cos(math.radians(mean_latitude)),
            1.0e-6,
        )
        squared_distances = [
            ((latitude - mean_latitude) * latitude_scale_m) ** 2
            + ((longitude - mean_longitude) * longitude_scale_m) ** 2
            for latitude, longitude in zip(latitudes, longitudes)
        ]
        return math.sqrt(statistics.fmean(squared_distances))


def main(args=None):
    rclpy.init(args=args)
    node = TopdownRtkLocalizerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
