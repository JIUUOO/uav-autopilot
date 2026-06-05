#!/usr/bin/env python3

import copy

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageFlipNode(Node):
    """Flip camera Image messages while preserving the original topic contract."""

    def __init__(self):
        super().__init__("image_flip")

        self.declare_parameter("input_topic", "/uav/camera/front/image_raw_unflipped")
        self.declare_parameter("output_topic", "/uav/camera/front/image_raw")
        self.declare_parameter("flip_mode", "rotate_180")  # none, vertical, horizontal, rotate_180

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.flip_mode = str(self.get_parameter("flip_mode").value).strip().lower()

        if self.flip_mode not in {"none", "vertical", "horizontal", "rotate_180"}:
            raise ValueError("flip_mode must be one of: none, vertical, horizontal, rotate_180")

        self.create_subscription(Image, self.input_topic, self.on_image, 10)
        self.pub = self.create_publisher(Image, self.output_topic, 10)

        self.get_logger().warn(
            f"Image flip started: input={self.input_topic}, output={self.output_topic}, mode={self.flip_mode}"
        )

    def on_image(self, msg):
        if self.flip_mode == "none":
            self.pub.publish(msg)
            return

        try:
            flipped = self.flip_image(msg)
        except Exception as exc:
            self.get_logger().error(f"Image flip failed: {exc}", throttle_duration_sec=5.0)
            return

        self.pub.publish(flipped)

    def flip_image(self, msg):
        bytes_per_pixel = self.bytes_per_pixel(msg.encoding)
        expected_step = msg.width * bytes_per_pixel
        if msg.step != expected_step:
            raise ValueError(
                f"Unsupported padded image step: step={msg.step}, expected={expected_step}"
            )

        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height,
            msg.width,
            bytes_per_pixel,
        )

        if self.flip_mode == "vertical":
            image = image[::-1, :, :]
        elif self.flip_mode == "horizontal":
            image = image[:, ::-1, :]
        elif self.flip_mode == "rotate_180":
            image = image[::-1, ::-1, :]

        flipped = copy.copy(msg)
        flipped.data = image.copy().tobytes()
        return flipped

    @staticmethod
    def bytes_per_pixel(encoding):
        encoding = encoding.lower()
        if encoding in {"mono8", "8uc1"}:
            return 1
        if encoding in {"rgb8", "bgr8", "8uc3"}:
            return 3
        if encoding in {"rgba8", "bgra8", "8uc4"}:
            return 4
        raise ValueError(f"Unsupported image encoding for flip: {encoding}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageFlipNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
