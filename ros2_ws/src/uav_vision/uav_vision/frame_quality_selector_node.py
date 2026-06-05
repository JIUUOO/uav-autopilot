#!/usr/bin/env python3

import copy
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from uav_interfaces.msg import FrameQuality

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None


class FrameQualitySelectorNode(Node):
    """
    1. Subscribe to "raw camera frames" and "IMU yaw-rate(=angular velocity)" telemetry.
    2. Score sampled frames using lightweight OpenCV quality metrics.
    3. Keep the "best non-rejected frame" inside each selection window.
    4. Publish both frame-quality telemetry and the selected image.
    """

    def __init__(self):
        super().__init__("frame_quality_selector")

        self.declare_parameter("image_topic", "/uav/camera/gimbal/image_raw")
        self.declare_parameter("imu_topic", "/mavros/imu/data")
        self.declare_parameter("selected_image_topic", "/uav/vision/selected_frame/image_raw")
        self.declare_parameter("quality_topic", "/uav/vision/frame_quality")
        self.declare_parameter("sample_hz", 5.0)
        self.declare_parameter("selection_window_sec", 5.0)
        self.declare_parameter("score_width", 160)
        self.declare_parameter("min_laplacian_var", 30.0)
        self.declare_parameter("min_brightness", 30.0)
        self.declare_parameter("max_brightness", 225.0)
        self.declare_parameter("max_saturation_ratio", 0.35)
        self.declare_parameter("max_yaw_rate_rad_s", 0.8)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.selected_image_topic = str(self.get_parameter("selected_image_topic").value)
        self.quality_topic = str(self.get_parameter("quality_topic").value)
        self.sample_hz = float(self.get_parameter("sample_hz").value)
        self.selection_window_sec = float(self.get_parameter("selection_window_sec").value)
        self.score_width = int(self.get_parameter("score_width").value)
        self.min_laplacian_var = float(self.get_parameter("min_laplacian_var").value)
        self.min_brightness = float(self.get_parameter("min_brightness").value)
        self.max_brightness = float(self.get_parameter("max_brightness").value)
        self.max_saturation_ratio = float(self.get_parameter("max_saturation_ratio").value)
        self.max_yaw_rate_rad_s = float(self.get_parameter("max_yaw_rate_rad_s").value)

        if cv2 is None or np is None:
            self.get_logger().error(
                "Missing OpenCV or NumPy. Install python3-opencv and python3-numpy."
            )

        self.latest_image = None
        self.latest_yaw_rate_rad_s = 0.0
        self.previous_selected_gray = None
        self.best_image = None
        self.best_quality = None
        self.best_score = None
        self.best_gray = None
        self.window_index = 0
        self.window_start_time = time.monotonic()

        self.create_subscription(Image, self.image_topic, self.on_image, 10)
        self.create_subscription(Imu, self.imu_topic, self.on_imu, 10)
        self.selected_image_pub = self.create_publisher(Image, self.selected_image_topic, 10)
        self.quality_pub = self.create_publisher(FrameQuality, self.quality_topic, 10)

        timer_period = 1.0 / max(self.sample_hz, 0.1)
        self.timer = self.create_timer(timer_period, self.on_timer)

        self.get_logger().warn(
            f"Frame quality selector started: image={self.image_topic}, "
            f"imu={self.imu_topic}, sample_hz={self.sample_hz}, "
            f"window={self.selection_window_sec}s"
        )

    def on_image(self, msg):
        self.latest_image = msg

    def on_imu(self, msg):
        self.latest_yaw_rate_rad_s = float(msg.angular_velocity.z)

    def on_timer(self):
        if cv2 is None or np is None:
            return

        msg = self.latest_image
        if msg is None:
            self.get_logger().warn("No image received yet.", throttle_duration_sec=10.0)
            return

        try:
            quality, score_gray = self.evaluate_frame(msg)
        except Exception as exc:
            self.get_logger().error(f"Frame quality evaluation failed: {exc}")
            return

        self.quality_pub.publish(quality)

        if not quality.rejected and (self.best_score is None or quality.total_score > self.best_score):
            self.best_score = float(quality.total_score)
            self.best_quality = copy.deepcopy(quality)
            self.best_image = copy.deepcopy(msg)
            self.best_gray = score_gray

        if time.monotonic() - self.window_start_time >= self.selection_window_sec:
            self.publish_best_frame()
            self.reset_window()

    def evaluate_frame(self, msg):
        rgb = self.image_msg_to_rgb_array(msg)
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        gray_small = self.resize_for_score(gray)

        laplacian_var = float(cv2.Laplacian(gray_small, cv2.CV_64F).var())
        brightness = float(gray_small.mean())
        saturation_ratio = float(
            np.mean((gray_small <= 5) | (gray_small >= 250))
        )
        scene_change_score = self.compute_scene_change_score(gray_small)
        yaw_rate_abs = abs(self.latest_yaw_rate_rad_s)

        blur_score = min(laplacian_var / max(self.min_laplacian_var * 4.0, 1.0), 1.0)
        brightness_score = 1.0 - min(abs(brightness - 127.5) / 127.5, 1.0)
        saturation_score = 1.0 - min(
            saturation_ratio / max(self.max_saturation_ratio, 0.001), 1.0
        )
        motion_score = 1.0 - min(yaw_rate_abs / max(self.max_yaw_rate_rad_s, 0.001), 1.0)

        total_score = (
            0.40 * blur_score
            + 0.30 * brightness_score
            + 0.20 * scene_change_score
            + 0.10 * motion_score
        )

        rejected, reject_reason = self.reject_reason(
            laplacian_var=laplacian_var,
            brightness=brightness,
            saturation_ratio=saturation_ratio,
            yaw_rate_abs=yaw_rate_abs,
        )

        quality = FrameQuality()
        quality.header = msg.header
        quality.image_source_topic = self.image_topic
        quality.window_index = self.window_index
        quality.blur_score = float(laplacian_var)
        quality.brightness = float(brightness)
        quality.saturation_ratio = float(saturation_ratio)
        quality.scene_change_score = float(scene_change_score)
        quality.yaw_rate_rad_s = float(self.latest_yaw_rate_rad_s)
        quality.total_score = float(total_score)
        quality.rejected = rejected
        quality.reject_reason = reject_reason
        quality.selected = False

        return quality, gray_small

    def reject_reason(self, *, laplacian_var, brightness, saturation_ratio, yaw_rate_abs):
        reasons = []
        if laplacian_var < self.min_laplacian_var:
            reasons.append("blur")
        if brightness < self.min_brightness:
            reasons.append("dark")
        if brightness > self.max_brightness:
            reasons.append("bright")
        if saturation_ratio > self.max_saturation_ratio:
            reasons.append("saturated")
        if yaw_rate_abs > self.max_yaw_rate_rad_s:
            reasons.append("yaw_rate")

        return bool(reasons), ",".join(reasons)

    def compute_scene_change_score(self, gray_small):
        if self.previous_selected_gray is None:
            return 1.0

        diff = cv2.absdiff(gray_small, self.previous_selected_gray)
        return float(min(diff.mean() / 50.0, 1.0))

    def publish_best_frame(self):
        if self.best_image is None or self.best_quality is None:
            self.get_logger().warn("No acceptable frame selected in this window.")
            return

        selected_quality = copy.deepcopy(self.best_quality)
        selected_quality.selected = True

        self.selected_image_pub.publish(self.best_image)
        self.quality_pub.publish(selected_quality)
        self.previous_selected_gray = self.best_gray

        self.get_logger().info(
            f"Selected frame: window={self.window_index} "
            f"score={selected_quality.total_score:.3f} "
            f"blur={selected_quality.blur_score:.1f} "
            f"brightness={selected_quality.brightness:.1f} "
            f"yaw_rate={selected_quality.yaw_rate_rad_s:.3f}"
        )

    def reset_window(self):
        self.window_index += 1
        self.window_start_time = time.monotonic()
        self.best_image = None
        self.best_quality = None
        self.best_score = None
        self.best_gray = None

    def resize_for_score(self, gray):
        if gray.shape[1] <= self.score_width:
            return gray

        scale = self.score_width / float(gray.shape[1])
        height = max(1, int(gray.shape[0] * scale))
        return cv2.resize(gray, (self.score_width, height), interpolation=cv2.INTER_AREA)

    @staticmethod
    def image_msg_to_rgb_array(msg):
        encoding = msg.encoding.lower()
        data = np.frombuffer(bytes(msg.data), dtype=np.uint8)

        formats = {
            "rgb8": (3, None),
            "bgr8": (3, cv2.COLOR_BGR2RGB),
            "rgba8": (4, cv2.COLOR_RGBA2RGB),
            "bgra8": (4, cv2.COLOR_BGRA2RGB),
            "mono8": (1, cv2.COLOR_GRAY2RGB),
        }
        if encoding not in formats:
            raise RuntimeError(f"Unsupported image encoding: {msg.encoding}")

        channels, conversion = formats[encoding]
        expected_step = msg.width * channels
        if msg.step < expected_step:
            raise RuntimeError(f"Invalid image step: step={msg.step}, expected>={expected_step}")

        rows = []
        for row in range(msg.height):
            start = row * msg.step
            rows.append(data[start:start + expected_step])

        image = np.concatenate(rows).reshape((msg.height, msg.width, channels))
        if channels == 1:
            image = image.reshape((msg.height, msg.width))

        if conversion is not None:
            image = cv2.cvtColor(image, conversion)

        return image


def main(args=None):
    rclpy.init(args=args)
    node = FrameQualitySelectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
