import math
import os
import time

import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend for headless environments
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Enable 3D projection support
import numpy as np
import ydlidar


# LiDAR vertical filtered point cloud generator
# Hardware: YDLIDAR T-mini Plus / MTF-02P
# Interface: USB-UART (/dev/ttyUSB0)
#
# Purpose:
#   Stack filtered vertical 2D LiDAR slices along a known straight path
#   and save a 3D point cloud.
#
# Output:
#   1. dev/maps/lidar_vertical_filtered_cloud.ply
#       $ sudo apt install -y cloudcompare
#       $ CloudCompare dev/maps/lidar_vertical_filtered_cloud.ply
#       To visualize point cloud 
#   2. dev/maps/lidar_vertical_filtered_cloud_preview.png
#
# Important:
#   This is the first indoor test.
#   It assumes the LiDAR/drone is moved forward at a roughly constant speed.
#   This is NOT Pixhawk-pose-based mapping yet.


# =========================
# LiDAR settings
# =========================

SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 230400
SCAN_FREQ = 4.0

MIN_RANGE = 0.10
MAX_RANGE = 30.0


# =========================
# Terrain-sector filter settings
# =========================

KEEP_ANGLE_MIN_DEG = -170.0
KEEP_ANGLE_MAX_DEG = -10.0
# Keep only the lower / terrain-facing sector.
#
# If this keeps the wrong side, try:
#   KEEP_ANGLE_MIN_DEG = 10.0
#   KEEP_ANGLE_MAX_DEG = 170.0

MAX_UPPER_Z = 0.30
# Remove upper-side arc artifacts.
#
# More aggressive:
#   MAX_UPPER_Z = 0.0


# =========================
# Known-path stacking settings
# =========================

NUM_SLICES = 80
# Number of vertical scans to stack.
#
# At SCAN_FREQ = 4 Hz:
#   80 slices ≈ 20 seconds of logging.

PATH_STEP_M = 0.05
# Assumed movement distance between consecutive slices.
#
# Example:
#   0.05 m/scan × 4 scans/sec = 0.20 m/sec assumed walking speed.
#
# If you move faster, increase this.
# If you move slower, decrease this.

START_DELAY_SEC = 3.0
# Delay before recording starts.
# Use this time to start moving steadily.


# =========================
# Output settings
# =========================

OUT_DIR = "dev/maps"
PLY_PATH = os.path.join(OUT_DIR, "lidar_vertical_filtered_cloud.ply")
PREVIEW_PNG = os.path.join(OUT_DIR, "lidar_vertical_filtered_cloud_preview.png")


def ensure_output_dir():
    os.makedirs(OUT_DIR, exist_ok=True)


def init_lidar():
    ydlidar.os_init()

    laser = ydlidar.CYdLidar()

    laser.setlidaropt(ydlidar.LidarPropSerialPort, SERIAL_PORT)
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, BAUD_RATE)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)

    laser.setlidaropt(ydlidar.LidarPropScanFrequency, SCAN_FREQ)
    laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
    laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
    laser.setlidaropt(ydlidar.LidarPropMinRange, MIN_RANGE)
    laser.setlidaropt(ydlidar.LidarPropMaxRange, MAX_RANGE)
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 20)
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)

    print(f"Connecting LiDAR: {SERIAL_PORT} @ {BAUD_RATE}")

    if not laser.initialize():
        raise RuntimeError("LiDAR initialize failed")

    if not laser.turnOn():
        laser.disconnecting()
        raise RuntimeError("LiDAR turnOn failed")

    print("LiDAR scanning started")
    return laser


def is_angle_in_sector(angle_deg):
    return KEEP_ANGLE_MIN_DEG <= angle_deg <= KEEP_ANGLE_MAX_DEG


def collect_one_filtered_slice(laser):
    scan = ydlidar.LaserScan()

    while ydlidar.os_isOk():
        ok = laser.doProcessSimple(scan)

        if not ok:
            time.sleep(0.05)
            continue

        points = scan.points
        n = points.size()

        slice_points = []

        raw_count = n
        valid_count = 0
        range_reject_count = 0
        sector_reject_count = 0
        upper_reject_count = 0

        for i in range(n):
            p = points[i]

            r = float(p.range)
            a = float(p.angle)

            if not math.isfinite(r):
                continue

            if r < MIN_RANGE or r > MAX_RANGE:
                range_reject_count += 1
                continue

            deg = math.degrees(a)

            if not is_angle_in_sector(deg):
                sector_reject_count += 1
                continue

            # Polar point -> vertical scan-plane Cartesian point.
            #
            # x_scan = horizontal axis in the LiDAR scan plane
            # z_scan = vertical axis in the LiDAR scan plane
            x_scan = r * math.cos(a)
            z_scan = r * math.sin(a)

            if z_scan > MAX_UPPER_Z:
                upper_reject_count += 1
                continue

            slice_points.append((x_scan, z_scan))
            valid_count += 1

        stats = {
            "raw": raw_count,
            "valid": valid_count,
            "range_reject": range_reject_count,
            "sector_reject": sector_reject_count,
            "upper_reject": upper_reject_count,
        }

        return np.array(slice_points, dtype=np.float32), stats

    return np.empty((0, 2), dtype=np.float32), {}


def build_known_path_cloud(laser):
    cloud_points = []

    print(f"Start delay: {START_DELAY_SEC:.1f}s")
    print("Move the drone/LiDAR forward in a straight line at a steady speed.")
    time.sleep(START_DELAY_SEC)

    print(f"Collecting {NUM_SLICES} vertical slices...")

    for slice_idx in range(NUM_SLICES):
        points_xz, stats = collect_one_filtered_slice(laser)

        # Known-path assumption:
        #   world_x = movement direction
        #   world_y = LiDAR scan horizontal axis
        #   world_z = LiDAR scan vertical axis
        #
        # This is a first test before using Pixhawk pose.
        world_x = slice_idx * PATH_STEP_M

        for x_scan, z_scan in points_xz:
            world_y = float(x_scan)
            world_z = float(z_scan)
            cloud_points.append((world_x, world_y, world_z))

        print(
            f"[slice {slice_idx + 1:03d}/{NUM_SLICES}] "
            f"x={world_x:.2f}m | "
            f"raw={stats.get('raw', 0)}, "
            f"valid={stats.get('valid', 0)}, "
            f"range_reject={stats.get('range_reject', 0)}, "
            f"sector_reject={stats.get('sector_reject', 0)}, "
            f"upper_reject={stats.get('upper_reject', 0)}"
        )

    return np.array(cloud_points, dtype=np.float32)


def save_ply(points_xyz, out_path):
    if len(points_xyz) == 0:
        print("No points to save")
        return

    with open(out_path, "w") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points_xyz)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")

        for x, y, z in points_xyz:
            f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")

    print(f"Saved: {out_path}")


def save_preview(points_xyz, out_path):
    if len(points_xyz) == 0:
        print("No points to preview")
        return

    # Downsample preview if the point cloud is large.
    max_preview_points = 50000

    if len(points_xyz) > max_preview_points:
        idx = np.linspace(0, len(points_xyz) - 1, max_preview_points).astype(np.int32)
        preview = points_xyz[idx]
    else:
        preview = points_xyz

    x = preview[:, 0]
    y = preview[:, 1]
    z = preview[:, 2]

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")

    ax.scatter(x, y, z, s=1)

    ax.set_xlabel("path x (m)")
    ax.set_ylabel("scan y (m)")
    ax.set_zlabel("scan z (m)")
    ax.set_title("Known-Path Vertical LiDAR Point Cloud")

    # Keep aspect roughly comparable.
    x_range = max(float(x.max() - x.min()), 1e-6)
    y_range = max(float(y.max() - y.min()), 1e-6)
    z_range = max(float(z.max() - z.min()), 1e-6)
    max_range = max(x_range, y_range, z_range)

    x_mid = float((x.max() + x.min()) / 2.0)
    y_mid = float((y.max() + y.min()) / 2.0)
    z_mid = float((z.max() + z.min()) / 2.0)

    ax.set_xlim(x_mid - max_range / 2.0, x_mid + max_range / 2.0)
    ax.set_ylim(y_mid - max_range / 2.0, y_mid + max_range / 2.0)
    ax.set_zlim(z_mid - max_range / 2.0, z_mid + max_range / 2.0)

    plt.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close()

    print(f"Saved: {out_path}")


def main():
    ensure_output_dir()

    laser = None

    try:
        laser = init_lidar()

        points_xyz = build_known_path_cloud(laser)

        print(f"Total cloud points: {len(points_xyz)}")

        save_ply(points_xyz, PLY_PATH)
        save_preview(points_xyz, PREVIEW_PNG)

        print("Done")

    except KeyboardInterrupt:
        print("\nStopped by user")

    finally:
        if laser is not None:
            laser.turnOff()
            laser.disconnecting()
            print("LiDAR stopped")


if __name__ == "__main__":
    main()
