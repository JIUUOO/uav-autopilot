import time
import math
import ydlidar  # Installed from external/YDLidar-SDK


# LiDAR simple monitor
# Hardware: YDLIDAR T-mini Plus


# Python Script → YDLidar-SDK → USB UART → YDLIDAR T-mini Plus
#
# A 2D LiDAR scan is a set of polar points:
#   point = (angle, range)
#
#   angle: beam direction, in radians
#   range: measured distance, in meters
#
# Example:
#   angle = 0.52 rad, range = 1.2 m
#   means: an object was detected 1.2 m away at about 30 degrees.

# =========================
# Serial / LiDAR settings
# =========================

SERIAL_PORT = "/dev/ttyUSB0"
# Linux serial port for the CP2102 USB-UART adapter.
# Check with:
#   ls -l /dev/ttyUSB* /dev/ttyACM*

BAUD_RATE = 230400 # T-mini Plus
SCAN_FREQ = 6.0 # Hz

# =========================
# SDK initialization
# =========================

ydlidar.os_init()
laser = ydlidar.CYdLidar()
laser.setlidaropt(ydlidar.LidarPropSerialPort, SERIAL_PORT)
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, BAUD_RATE)
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF)
# Set the LiDAR measurement type.
# TYPE_TOF = Time-of-Flight LiDAR.

laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
laser.setlidaropt(ydlidar.LidarPropScanFrequency, SCAN_FREQ)
laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
# Set the angular range to use, in degrees.
# -180 to +180 means full 360-degree scanning.

laser.setlidaropt(ydlidar.LidarPropMinRange, 0.05)
laser.setlidaropt(ydlidar.LidarPropMaxRange, 12.0)
# Set the valid distance range, in meters.
# Points outside this range are ignored by the SDK.

laser.setlidaropt(ydlidar.LidarPropSampleRate, 20)
# Sampling rate setting for the LiDAR model.
# This affects how many distance samples are produced per scan.

laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
# Single-channel protocol option.
# False is used here for the current T-mini Plus setup.


print(f"Connecting LiDAR: {SERIAL_PORT} @ {BAUD_RATE}")

# =========================
# Connect and start scanning
# =========================

ret = laser.initialize()
# Open the serial connection and initialize the LiDAR.

if not ret:
    print("LiDAR initialize failed")
    raise SystemExit(1)

ret = laser.turnOn()
# Start LiDAR scanning.
# The motor/scan mode starts after this call.

if not ret:
    print("LiDAR turnOn failed")
    laser.disconnecting()
    raise SystemExit(1)

print("LiDAR scanning started")
print("Press Ctrl+C to stop")


# =========================
# Scan loop
# =========================

scan = ydlidar.LaserScan()
# LaserScan stores one scan frame.
# One scan frame contains many points.


try:
    while ydlidar.os_isOk():
        ret = laser.doProcessSimple(scan)
        # Read one scan frame from the LiDAR.
        # If successful, scan.points will contain the measured points.

        if not ret:
            print("No scan")
            time.sleep(0.1)
            continue

        points = scan.points
        n = points.size()
        # Number of raw points in this scan frame.

        valid = []

        for i in range(n):
            p = points[i]

            r = float(p.range)
            # Distance in meters.

            a = float(p.angle)
            # Angle in radians.

            if not math.isfinite(r):
                continue
            # Reject invalid numeric values:
            #   NaN, +inf, -inf
            #
            # These values can break min(), coordinate transforms,
            # and map generation.

            if r < 0.05 or r > 12.0:
                continue
            # Reject points outside the usable distance range.
            #
            # r < 0.05:
            #   Too close. Often unreliable or outside the sensor's valid range.
            #
            # r > 12.0:
            #   Too far for this test setup.
            #
            # This is a software-side safety filter, even though similar limits
            # were already set in the SDK options.

            valid.append((r, a))
            # Store only valid polar points:
            #   r = range in meters
            #   a = angle in radians

        if not valid:
            print(f"points={n} | valid=0")
            continue

        min_r, min_a = min(valid, key=lambda x: x[0])
        # Find the nearest detected point in the full 360-degree scan.

        front = [
            r for r, a in valid
            if abs(math.degrees(a)) <= 10.0
        ]
        # Extract points near the sensor's forward direction.
        #
        # Here, "front" means:
        #   -10 degrees <= angle <= +10 degrees
        #
        # This is useful for quick obstacle checks.
        #
        # Important:
        #   The LiDAR's 0-degree direction must physically match
        #   the robot/drone's forward direction for this to be meaningful.

        front_min = min(front) if front else None
        # Nearest object distance in the forward ±10-degree sector.

        if front_min is None:
            front_text = "None"
        else:
            front_text = f"{front_min:.3f}m"

        print(
            f"points={n:4d} | "
            f"valid={len(valid):4d} | "
            f"min={min_r:.3f}m @ {math.degrees(min_a):+.1f}deg | "
            f"front_min={front_text}"
        )
        # Example output:
        #
        # points= 620 | valid= 580 | min=0.432m @ -22.1deg | front_min=1.204m
        #
        # Meaning:
        #   points:
        #       raw number of points in the scan frame
        #
        #   valid:
        #       number of points after filtering invalid/out-of-range values
        #
        #   min:
        #       closest object in the full scan
        #
        #   front_min:
        #       closest object near the forward direction

except KeyboardInterrupt:
    print("\nStopped by user")

finally:
    laser.turnOff()
    laser.disconnecting()
    print("LiDAR stopped")