#!/usr/bin/env python3
"""Render a ROS 2 flight bag as a split-view camera and telemetry MP4."""

import argparse
from pathlib import Path
import subprocess
import sys

import cv2
import numpy as np

if __package__:
    from .bag_reader import load_bag
    from .dashboard_renderer import GraphRenderer, draw_osd
    from .frame_decoder import raw_to_bgr
else:
    from bag_reader import load_bag
    from dashboard_renderer import GraphRenderer, draw_osd
    from frame_decoder import raw_to_bgr


def apply_camera_flip(frame: np.ndarray, flip_mode: str) -> np.ndarray:
    """Apply the same camera flip modes supported by the ROS image flip node."""
    if flip_mode == "vertical":
        return cv2.flip(frame, 0)
    if flip_mode == "horizontal":
        return cv2.flip(frame, 1)
    if flip_mode == "rotate_180":
        return cv2.flip(frame, -1)
    return frame


def render(args):
    """Render the selected bag into a split-view MP4."""
    frames, store = load_bag(
        args.bag,
        args.cam_topic,
        args.alt_topic,
        args.batt_topic,
        args.state_topic,
    )
    if not frames:
        print("[error] No camera frames found. Check --cam-topic.")
        sys.exit(1)

    width = args.width
    height = args.height
    total_width = width * 2
    output = args.output or f"flight_{Path(args.bag).stem}.mp4"
    print(f"[render] Output: {output} ({total_width}x{height} @ {args.fps}fps)")

    graph_renderer = GraphRenderer(width, height, args.graph_sec)
    process = None
    writer = None
    if not args.no_ffmpeg:
        process = subprocess.Popen(
            [
                "ffmpeg",
                "-y",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "bgr24",
                "-s",
                f"{total_width}x{height}",
                "-r",
                str(args.fps),
                "-i",
                "pipe:0",
                "-c:v",
                "libx264",
                "-preset",
                args.preset,
                "-crf",
                str(args.crf),
                "-pix_fmt",
                "yuv420p",
                output,
            ],
            stdin=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
    else:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(
            output, fourcc, args.fps, (total_width, height)
        )

    start_time = frames[0][0]
    for index, (timestamp, raw, encoding, frame_height, frame_width) in enumerate(
        frames
    ):
        try:
            camera = raw_to_bgr(raw, encoding, frame_height, frame_width)
        except Exception as error:
            print(f"[warn] Frame {index} decode error: {error}")
            camera = np.zeros((height, width, 3), dtype=np.uint8)

        camera = apply_camera_flip(camera, args.usb_cam_flip_mode)
        camera = cv2.resize(camera, (width, height))
        camera = draw_osd(
            camera, store.current_telem(timestamp), timestamp - start_time
        )
        graphs = graph_renderer.render(store, timestamp)
        composite = np.concatenate([camera, graphs], axis=1)

        if process:
            process.stdin.write(composite.tobytes())
        else:
            writer.write(composite)

        if index % 50 == 0:
            percentage = (index + 1) / len(frames) * 100
            print(
                f"  [{percentage:5.1f}%] frame {index + 1}/{len(frames)} "
                f"t={timestamp - start_time:.1f}s"
            )

    if process:
        process.stdin.close()
        return_code = process.wait()
        if return_code:
            raise RuntimeError(f"ffmpeg exited with status {return_code}")
    else:
        writer.release()
    print(f"\n[done] Saved: {output}")


def parse_args():
    """Parse command-line options."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="Path to bag directory or .db3 file")
    parser.add_argument("--output", default="", help="Output MP4 path")
    parser.add_argument("--cam-topic", default="/uav/camera/gimbal/image_raw")
    parser.add_argument(
        "--usb-cam-flip-mode",
        default="none",
        choices=["none", "vertical", "horizontal", "rotate_180"],
        help="Camera flip mode matching the ROS usb_cam_flip_mode parameter",
    )
    parser.add_argument("--alt-topic", default="/mavros/global_position/global")
    parser.add_argument("--batt-topic", default="/mavros/battery")
    parser.add_argument("--state-topic", default="/mavros/state")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--graph-sec", type=float, default=30.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--no-ffmpeg", action="store_true")
    parser.add_argument(
        "--preset",
        default="fast",
        choices=[
            "ultrafast",
            "superfast",
            "veryfast",
            "faster",
            "fast",
            "medium",
            "slow",
        ],
    )
    parser.add_argument("--crf", type=int, default=22)
    return parser.parse_args()


def main():
    render(parse_args())


if __name__ == "__main__":
    main()
