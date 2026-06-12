#!/usr/bin/env python3

from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from uav_interfaces.msg import GeminiReport
from uav_interfaces.msg import TrackedTarget
from uav_vision.common.gemini_utils import find_candidate
from uav_vision.common.image_utils import image_msg_to_bgr

try:
    import cv2
except ImportError:
    cv2 = None


class TargetTrackerNode(Node):
    """
    Initialize an OpenCV tracker from Gemini's timestamped primary bounding box.

    Raw frames are buffered so a delayed Gemini report can initialize the tracker
    on the exact image that Gemini analyzed, then tracking continues on newer frames.
    """

    def __init__(self):
        super().__init__("target_tracker")

        self.declare_parameter("image_topic", "/uav/camera/gimbal/image_raw")
        self.declare_parameter("gemini_report_topic", "/uav/vision/gemini_report")
        self.declare_parameter("tracked_target_topic", "/uav/vision/tracked_target")
        self.declare_parameter("tracker_type", "AUTO")
        self.declare_parameter("min_detection_confidence", 0.50)
        self.declare_parameter("max_buffer_frames", 300)
        self.declare_parameter("max_replay_frames", 60)
        self.declare_parameter("stamp_tolerance_sec", 0.05)
        self.declare_parameter("max_consecutive_failures", 5)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.gemini_report_topic = str(self.get_parameter("gemini_report_topic").value)
        self.tracked_target_topic = str(
            self.get_parameter("tracked_target_topic").value
        )
        self.tracker_type = str(self.get_parameter("tracker_type").value).upper()
        self.min_detection_confidence = float(
            self.get_parameter("min_detection_confidence").value
        )
        self.max_buffer_frames = max(
            int(self.get_parameter("max_buffer_frames").value),
            1,
        )
        self.max_replay_frames = max(
            int(self.get_parameter("max_replay_frames").value),
            1,
        )
        self.stamp_tolerance_sec = max(
            float(self.get_parameter("stamp_tolerance_sec").value),
            0.0,
        )
        self.max_consecutive_failures = max(
            int(self.get_parameter("max_consecutive_failures").value),
            1,
        )

        if cv2 is None:
            raise RuntimeError("Missing OpenCV. Install/rebuild Docker image first.")

        self.frame_buffer = deque(maxlen=self.max_buffer_frames)
        self.tracker = None
        self.active_tracker_type = ""
        self.initialization_stamp_ns = -1
        self.initialization_request_id = ""
        self.initialization_call_index = 0
        self.initialization_candidate_index = -1
        self.target_label = ""
        self.detection_confidence = 0.0
        self.tracked_frames = 0
        self.consecutive_failures = 0

        self.create_subscription(Image, self.image_topic, self.on_image, 10)
        self.create_subscription(
            GeminiReport,
            self.gemini_report_topic,
            self.on_gemini_report,
            10,
        )
        self.tracked_target_pub = self.create_publisher(
            TrackedTarget,
            self.tracked_target_topic,
            10,
        )

        self.get_logger().warn(
            f"Target tracker started: image={self.image_topic}, "
            f"report={self.gemini_report_topic}, output={self.tracked_target_topic}, "
            f"tracker={self.tracker_type}, buffer_frames={self.max_buffer_frames}"
        )

    def on_image(self, msg):
        stamp_ns = self.stamp_ns(msg.header.stamp)
        self.frame_buffer.append((stamp_ns, msg))

        if self.tracker is None or stamp_ns <= self.initialization_stamp_ns:
            return

        self.track_frame(msg)

    def track_frame(self, msg):
        try:
            frame = image_msg_to_bgr(msg)
            success, bbox = self.tracker.update(frame)
        except Exception as exc:
            self.get_logger().error(f"Target tracker update failed: {exc}")
            self.publish_result(msg, tracking=False, status="tracker_update_error")
            self.reset_tracker()
            return

        if success:
            self.tracked_frames += 1
            self.consecutive_failures = 0
            self.publish_result(
                msg,
                tracking=True,
                status="tracking",
                bbox_pixels=bbox,
            )
            return

        self.consecutive_failures += 1
        self.publish_result(msg, tracking=False, status="tracking_lost")
        if self.consecutive_failures >= self.max_consecutive_failures:
            self.get_logger().warn(
                "Target tracking stopped after "
                f"{self.consecutive_failures} consecutive failures."
            )
            self.reset_tracker()

    def on_gemini_report(self, report):
        if not report.parsed_ok or not report.target_detected:
            return

        candidate = find_candidate(
            report.target_candidates,
            report.primary_candidate_index,
        )
        if candidate is None or candidate.confidence < self.min_detection_confidence:
            return

        source_frame = self.find_buffered_frame(report.header.stamp)
        if source_frame is None:
            self.get_logger().warn(
                f"Cannot initialize tracker: source frame is no longer buffered "
                f"for request={report.request_id}."
            )
            return

        try:
            frame = image_msg_to_bgr(source_frame)
            bbox = self.normalized_to_pixel_bbox(candidate, frame.shape)
            tracker, tracker_type = self.create_tracker()
            initialized = tracker.init(frame, bbox)
            if initialized is False:
                raise RuntimeError("OpenCV tracker rejected the initial bounding box.")
        except Exception as exc:
            self.get_logger().error(f"Target tracker initialization failed: {exc}")
            return

        self.tracker = tracker
        self.active_tracker_type = tracker_type
        self.initialization_stamp_ns = self.stamp_ns(source_frame.header.stamp)
        self.initialization_request_id = report.request_id
        self.initialization_call_index = int(report.call_index)
        self.initialization_candidate_index = int(candidate.candidate_index)
        self.target_label = candidate.target_label
        self.detection_confidence = float(candidate.confidence)
        self.tracked_frames = 0
        self.consecutive_failures = 0

        self.publish_result(
            source_frame,
            tracking=True,
            status="initialized",
            bbox_pixels=bbox,
        )
        self.replay_buffered_frames()
        self.get_logger().warn(
            f"Target tracker initialized: request={report.request_id}, "
            f"label={self.target_label!r}, tracker={self.active_tracker_type}"
        )

    def replay_buffered_frames(self):
        """Advance a newly initialized tracker through frames received during Gemini latency."""

        newer_frames = [
            msg
            for stamp_ns, msg in self.frame_buffer
            if stamp_ns > self.initialization_stamp_ns
        ]
        if len(newer_frames) > self.max_replay_frames:
            if self.max_replay_frames == 1:
                newer_frames = [newer_frames[-1]]
            else:
                last_index = len(newer_frames) - 1
                newer_frames = [
                    newer_frames[
                        round(index * last_index / (self.max_replay_frames - 1))
                    ]
                    for index in range(self.max_replay_frames)
                ]

        for msg in newer_frames:
            if self.tracker is None:
                break
            self.track_frame(msg)

    def find_buffered_frame(self, stamp):
        target_ns = self.stamp_ns(stamp)
        tolerance_ns = int(self.stamp_tolerance_sec * 1_000_000_000)
        closest = None
        closest_delta_ns = None

        for stamp_ns, msg in self.frame_buffer:
            delta_ns = abs(stamp_ns - target_ns)
            if closest_delta_ns is None or delta_ns < closest_delta_ns:
                closest = msg
                closest_delta_ns = delta_ns

        if closest_delta_ns is None or closest_delta_ns > tolerance_ns:
            return None
        return closest

    def create_tracker(self):
        requested = self.tracker_type
        candidates = (
            ["CSRT", "KCF", "MIL"]
            if requested == "AUTO"
            else [requested]
        )

        for name in candidates:
            factory_name = f"Tracker{name}_create"
            factory = getattr(cv2, factory_name, None)
            if factory is not None:
                return factory(), name

            legacy = getattr(cv2, "legacy", None)
            legacy_factory = getattr(legacy, factory_name, None) if legacy else None
            if legacy_factory is not None:
                return legacy_factory(), name

        raise RuntimeError(
            f"No supported OpenCV tracker factory found for {candidates}."
        )

    def publish_result(self, image_msg, *, tracking, status, bbox_pixels=None):
        msg = TrackedTarget()
        msg.header = image_msg.header
        msg.tracking = bool(tracking)
        msg.status = status
        msg.tracker_type = self.active_tracker_type
        msg.initialization_request_id = self.initialization_request_id
        msg.initialization_call_index = self.initialization_call_index
        msg.initialization_candidate_index = self.initialization_candidate_index
        msg.target_label = self.target_label
        msg.detection_confidence = self.detection_confidence
        msg.tracked_frames = self.tracked_frames
        msg.consecutive_failures = self.consecutive_failures

        if bbox_pixels is not None:
            x, y, width, height = self.clamp_pixel_bbox(
                bbox_pixels,
                image_msg.width,
                image_msg.height,
            )
            msg.bbox_x_min_norm = x / image_msg.width
            msg.bbox_y_min_norm = y / image_msg.height
            msg.bbox_x_max_norm = (x + width) / image_msg.width
            msg.bbox_y_max_norm = (y + height) / image_msg.height
            msg.center_x_norm = (x + width / 2.0) / image_msg.width
            msg.center_y_norm = (y + height / 2.0) / image_msg.height
            msg.bbox_area_norm = (width * height) / (
                image_msg.width * image_msg.height
            )

        self.tracked_target_pub.publish(msg)

    def reset_tracker(self):
        self.tracker = None
        self.active_tracker_type = ""
        self.initialization_stamp_ns = -1

    @staticmethod
    def normalized_to_pixel_bbox(candidate, shape):
        height, width = shape[:2]
        x_min = candidate.bbox_x_min_norm * width
        y_min = candidate.bbox_y_min_norm * height
        x_max = candidate.bbox_x_max_norm * width
        y_max = candidate.bbox_y_max_norm * height
        bbox = TargetTrackerNode.clamp_pixel_bbox(
            (x_min, y_min, x_max - x_min, y_max - y_min),
            width,
            height,
        )
        return tuple(int(round(value)) for value in bbox)

    @staticmethod
    def clamp_pixel_bbox(bbox, image_width, image_height):
        x, y, width, height = (float(value) for value in bbox)
        x = max(0.0, min(x, image_width - 1.0))
        y = max(0.0, min(y, image_height - 1.0))
        width = max(1.0, min(width, image_width - x))
        height = max(1.0, min(height, image_height - y))
        return x, y, width, height

    @staticmethod
    def stamp_ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def main(args=None):
    rclpy.init(args=args)
    node = TargetTrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
