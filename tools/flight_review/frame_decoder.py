"""Decode ROS camera messages into OpenCV BGR frames."""

import cv2
import numpy as np


def raw_to_bgr(data: bytes, encoding: str, height: int, width: int) -> np.ndarray:
    """Decode raw or compressed ROS image data into a BGR image."""
    enc = encoding.lower()

    if enc == "rgb8":
        image = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3)
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if enc in ("bgr8", "8uc3"):
        return np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3)
    if enc in ("mono8", "8uc1"):
        image = np.frombuffer(data, dtype=np.uint8).reshape(height, width)
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    # Some camera drivers put JPEG bytes in sensor_msgs/Image while reporting
    # a raw encoding, so always try compressed decoding as a fallback.
    image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is not None:
        return image

    raise ValueError(f"Unsupported encoding: {encoding}")
