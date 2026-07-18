import argparse
import math
import os
import tempfile
import time

os.environ.setdefault(
    "MPLCONFIGDIR", os.path.join(tempfile.gettempdir(), "uav-autopilot-matplotlib")
)
os.environ.setdefault(
    "XDG_CACHE_HOME", os.path.join(tempfile.gettempdir(), "uav-autopilot-cache")
)

import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
import numpy as np
import ydlidar


# Live horizontal 2D viewer for YDLIDAR Tmini Pro/Plus.
# Native scan coordinates are not modified. The rendered result has been
# rendered with the calibrated screen signs:
#   screen horizontal = sensor x
#   screen vertical   = -sensor y
# The forward arrow is a UI-only indicator fixed straight upward.

DEFAULT_PORT = "/dev/cu.usbserial-0001"
BAUD_RATE = 230400
SCAN_FREQ = 6.0
SAMPLE_RATE = 4

DEFAULT_MIN_RANGE = 0.10
DEFAULT_MAX_RANGE = 12.0
BASE_VIEW_RANGE = 4.0
DEFAULT_VIEW_RANGE = (BASE_VIEW_RANGE / 1.3 / 1.5 / 1.3) / 0.8
# Previous 2.535x zoom, then scale the displayed result down by 20%.
LAST_FRAME_PATH = "dev/maps/lidar_horizontal_live_last.png"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Display live horizontal YDLIDAR scans in an XY window."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="LiDAR serial port")
    parser.add_argument(
        "--min-range",
        type=float,
        default=DEFAULT_MIN_RANGE,
        help=f"minimum accepted range in metres (default: {DEFAULT_MIN_RANGE})",
    )
    parser.add_argument(
        "--max-range",
        type=float,
        default=DEFAULT_MAX_RANGE,
        help=f"maximum accepted range in metres (default: {DEFAULT_MAX_RANGE})",
    )
    parser.add_argument(
        "--view-range",
        type=float,
        default=DEFAULT_VIEW_RANGE,
        help=f"half-width of the displayed area in metres (default: {DEFAULT_VIEW_RANGE:.2f})",
    )
    parser.add_argument(
        "--point-size",
        type=float,
        default=8.0,
        help="marker size used for scan points (default: 8)",
    )
    parser.add_argument(
        "--no-save",
        action="store_true",
        help="do not save the final displayed frame when closing",
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=0,
        help="stop after this many frames; 0 runs until the window closes",
    )
    return parser.parse_args()


def validate_args(args):
    if args.min_range < 0.0 or args.max_range <= args.min_range:
        raise ValueError("range must satisfy 0 <= --min-range < --max-range")
    if args.view_range <= 0.0:
        raise ValueError("--view-range must be greater than zero")
    if args.point_size <= 0.0:
        raise ValueError("--point-size must be greater than zero")
    if args.frames < 0:
        raise ValueError("--frames must be zero or greater")


def init_lidar(port, min_range, max_range):
    ydlidar.os_init()
    laser = ydlidar.CYdLidar()

    laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, BAUD_RATE)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, SCAN_FREQ)
    laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
    laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
    laser.setlidaropt(ydlidar.LidarPropMinRange, min_range)
    laser.setlidaropt(ydlidar.LidarPropMaxRange, max_range)
    laser.setlidaropt(ydlidar.LidarPropSampleRate, SAMPLE_RATE)
    laser.setlidaropt(ydlidar.LidarPropIntenstiy, True)
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)

    print(f"Connecting LiDAR: {port} @ {BAUD_RATE}")

    if not laser.initialize():
        raise RuntimeError("LiDAR initialize failed")

    if not laser.turnOn():
        laser.disconnecting()
        raise RuntimeError("LiDAR turnOn failed")

    print("LiDAR scanning started")
    return laser


def scan_to_xy(scan, min_range, max_range):
    xy_points = []
    ranges = []

    for i in range(scan.points.size()):
        point = scan.points[i]
        distance = float(point.range)
        angle = float(point.angle)

        if not math.isfinite(distance) or not math.isfinite(angle):
            continue
        if distance < min_range or distance > max_range:
            continue

        xy_points.append(
            (
                distance * math.cos(angle),
                distance * math.sin(angle),
            )
        )
        ranges.append(distance)

    if not xy_points:
        return np.empty((0, 2), dtype=np.float32), np.empty(0, dtype=np.float32)

    return (
        np.asarray(xy_points, dtype=np.float32),
        np.asarray(ranges, dtype=np.float32),
    )


def create_view(args):
    plt.ion()
    figure, axis = plt.subplots(figsize=(9, 9))

    scatter = axis.scatter(
        [],
        [],
        c=[],
        s=args.point_size,
        cmap="viridis",
        norm=Normalize(vmin=args.min_range, vmax=args.max_range),
    )
    axis.scatter([0.0], [0.0], c="red", marker="x", s=100, label="LiDAR")
    axis.arrow(
        0.0,
        0.0,
        0.0,
        min(0.7, args.view_range * 0.2),
        width=max(0.01, args.view_range * 0.004),
        color="red",
    )
    axis.text(
        0.0,
        min(0.8, args.view_range * 0.23),
        "forward",
        color="red",
        ha="center",
    )

    axis.set_xlim(-args.view_range, args.view_range)
    axis.set_ylim(-args.view_range, args.view_range)
    axis.set_aspect("equal", adjustable="box")
    axis.grid(True, alpha=0.3)
    axis.set_xlabel("sensor x (m)")
    axis.set_ylabel("-sensor y (m)")
    axis.set_title("Live Horizontal LiDAR")
    axis.legend(loc="upper right")
    figure.colorbar(scatter, ax=axis, label="range (m)")
    figure.tight_layout()
    figure.show()

    return figure, axis, scatter


def run_live_view(laser, args):
    figure, axis, scatter = create_view(args)
    scan = ydlidar.LaserScan()
    frame_count = 0
    rendered_frames = 0
    fps = 0.0
    fps_window_start = time.monotonic()

    while ydlidar.os_isOk() and plt.fignum_exists(figure.number):
        if not laser.doProcessSimple(scan):
            figure.canvas.flush_events()
            time.sleep(0.02)
            continue

        points_xy, ranges = scan_to_xy(scan, args.min_range, args.max_range)
        # Apply only the calibrated screen signs. The underlying scan
        # coordinates stay unchanged: screen = (sensor x, -sensor y).
        display_points = np.column_stack((points_xy[:, 0], -points_xy[:, 1]))
        scatter.set_offsets(display_points)
        scatter.set_array(ranges)

        frame_count += 1
        rendered_frames += 1
        now = time.monotonic()
        elapsed = now - fps_window_start
        if elapsed >= 1.0:
            fps = frame_count / elapsed
            frame_count = 0
            fps_window_start = now

        axis.set_title(
            f"Live Horizontal LiDAR | points={len(points_xy)} | {fps:.1f} FPS"
        )
        figure.canvas.draw_idle()
        figure.canvas.flush_events()
        plt.pause(0.001)

        if args.frames and rendered_frames >= args.frames:
            break

    return figure


def main():
    args = parse_args()
    validate_args(args)
    laser = None
    figure = None

    try:
        laser = init_lidar(args.port, args.min_range, args.max_range)
        print("Close the window or press Ctrl+C to stop")
        figure = run_live_view(laser, args)

    except KeyboardInterrupt:
        print("\nStopped by user")

    finally:
        if figure is not None and not args.no_save:
            os.makedirs(os.path.dirname(LAST_FRAME_PATH), exist_ok=True)
            figure.savefig(LAST_FRAME_PATH, dpi=200, bbox_inches="tight")
            print(f"Saved: {LAST_FRAME_PATH}")

        if laser is not None:
            laser.turnOff()
            laser.disconnecting()
            print("LiDAR stopped")

        plt.ioff()
        plt.close("all")


if __name__ == "__main__":
    main()
