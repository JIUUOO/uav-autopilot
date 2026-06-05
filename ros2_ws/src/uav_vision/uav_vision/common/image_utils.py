"""Image conversion helpers shared by vision nodes."""

import io


def image_msg_to_pil(msg):
    """Convert a ROS Image message into a PIL image."""

    try:
        from PIL import Image as PILImage
    except ImportError as exc:
        raise RuntimeError("Missing Pillow. Install/rebuild Docker image first.") from exc

    encoding = msg.encoding.lower()
    data = bytes(msg.data)

    if data.startswith(b"\xff\xd8"):
        return PILImage.open(io.BytesIO(data)).convert("RGB")

    formats = {
        "rgb8": ("RGB", "RGB", 3),
        "bgr8": ("RGB", "BGR", 3),
        "rgba8": ("RGBA", "RGBA", 4),
        "bgra8": ("RGBA", "BGRA", 4),
        "mono8": ("L", "L", 1),
    }
    if encoding not in formats:
        raise RuntimeError(f"Unsupported image encoding: {msg.encoding}")

    mode, raw_mode, bytes_per_pixel = formats[encoding]
    expected_step = msg.width * bytes_per_pixel
    if msg.step < expected_step:
        raise RuntimeError(
            f"Invalid image step: step={msg.step}, expected>={expected_step}"
        )

    if msg.step == expected_step:
        raw = data[:expected_step * msg.height]
    else:
        rows = []
        for row in range(msg.height):
            start = row * msg.step
            rows.append(data[start:start + expected_step])
        raw = b"".join(rows)

    return PILImage.frombytes(mode, (msg.width, msg.height), raw, "raw", raw_mode)
