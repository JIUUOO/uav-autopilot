import math
import os
import time

import matplotlib.pyplot as plt
import numpy as np
import ydlidar  # Installed from external/YDLidar-SDK


# LiDAR vertical profile mapper
# Hardware: YDLIDAR T-mini Plus / MTF-02P
# Interface: USB-UART (/dev/ttyUSB0)
#
# This script is for a vertically mounted 2D LiDAR.
# It does NOT generate a floor-plan map.
#
# Output meaning:
#   x = scan-plane horizontal axis
#   z = scan-plane vertical axis
#
# It saves:
#   1. vertical_profile_points.png
#   2. vertical_profile_hit_map.png


# =========================
# LiDAR settings
# =========================

SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 230400
SCAN_FREQ = 4.0

NUM_SCANS = 1
# Use 1 scan first.
# If the LiDAR is perfectly fixed, you can increase this to 5~20.

MIN_RANGE = 0.10
MAX_RANGE = 30.0
# Reduce MAX_RANGE if you see far-distance arc artifacts.


# =========================
# Vertical profile map settings
# =========================

MAP_SIZE_M = 16.0
# 16m x 16m scan-plane map.
# Since the LiDAR is vertical, this is NOT floor-plan width/height.
# It is the size of the vertical scan plane.

MAP_RESOLUTION = 0.05
# 5 cm per grid cell.

POINTS_PNG = "dev/maps/vertical_profile_points.png"
MAP_PNG = "dev/maps/vertical_profile_hit_map.png"


def ensure_output_dir():
    os.makedirs("dev/maps", exist_ok=True)


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


def collect_vertical_points(laser, num_scans):
    scan = ydlidar.LaserScan()
    all_points = []

    collected = 0

    while collected < num_scans and ydlidar.os_isOk():
        ok = laser.doProcessSimple(scan)

        if not ok:
            print(f"[scan {collected + 1}/{num_scans}] no data")
            time.sleep(0.05)
            continue

        points = scan.points
        n = points.size()

        valid_count = 0

        for i in range(n):
            p = points[i]

            r = float(p.range)
            a = float(p.angle)

            if not math.isfinite(r):
                continue

            if r < MIN_RANGE or r > MAX_RANGE:
                continue

            # Polar point -> scan-plane Cartesian point.
            #
            # For a vertically mounted LiDAR:
            #   x = horizontal axis in the scan plane
            #   z = vertical axis in the scan plane
            #
            # The exact physical direction depends on how the LiDAR is mounted.
            x = r * math.cos(a)
            z = r * math.sin(a)

            all_points.append((x, z))
            valid_count += 1

        collected += 1
        print(f"[scan {collected}/{num_scans}] raw={n}, valid={valid_count}")

    return np.array(all_points, dtype=np.float32)


def save_points_plot(points_xz, out_path):
    if len(points_xz) == 0:
        print("No valid points to plot")
        return

    x = points_xz[:, 0]
    z = points_xz[:, 1]

    plt.figure(figsize=(8, 8))
    plt.scatter(x, z, s=2)
    plt.scatter([0], [0], s=60, marker="x")  # LiDAR origin
    plt.axis("equal")
    plt.grid(True)
    plt.xlabel("scan x (m)")
    plt.ylabel("scan z (m)")
    plt.title("LiDAR Vertical Scan Profile")
    plt.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close()

    print(f"Saved: {out_path}")


def build_hit_grid(points_xz, map_size_m, resolution):
    cells = int(map_size_m / resolution)
    grid = np.zeros((cells, cells), dtype=np.uint8)

    origin = cells // 2

    for x, z in points_xz:
        col = int(round(x / resolution)) + origin
        row = origin - int(round(z / resolution))

        if 0 <= row < cells and 0 <= col < cells:
            grid[row, col] = 255

    return grid


def save_hit_grid(grid, out_path):
    half_size = MAP_SIZE_M / 2.0

    plt.figure(figsize=(8, 8))
    plt.imshow(
        grid,
        cmap="gray",
        origin="upper",
        extent=[-half_size, half_size, -half_size, half_size],
    )
    plt.scatter([0], [0], s=60, marker="x")
    plt.axis("equal")
    plt.grid(True)
    plt.xlabel("scan x (m)")
    plt.ylabel("scan z (m)")
    plt.title(f"LiDAR Vertical Hit Map ({MAP_RESOLUTION:.2f} m/cell)")
    plt.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close()

    print(f"Saved: {out_path}")


def main():
    ensure_output_dir()

    laser = None

    try:
        laser = init_lidar()

        print(f"Collecting {NUM_SCANS} scan(s)...")
        points_xz = collect_vertical_points(laser, NUM_SCANS)

        print(f"Total valid points: {len(points_xz)}")

        save_points_plot(points_xz, POINTS_PNG)

        grid = build_hit_grid(
            points_xz,
            map_size_m=MAP_SIZE_M,
            resolution=MAP_RESOLUTION,
        )

        save_hit_grid(grid, MAP_PNG)

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
