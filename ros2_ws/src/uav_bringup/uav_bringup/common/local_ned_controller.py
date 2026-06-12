#!/usr/bin/env python3

import math
import time

from pymavlink import mavutil
import rclpy


# Position-only SET_POSITION_TARGET_LOCAL_NED: ignore velocity, acceleration, yaw, and yaw-rate.
POSITION_TARGET_TYPEMASK = 3576


class LocalNedController:
    """Shared LOCAL_NED telemetry, position-target, origin, and yaw control."""

    def __init__(
        self,
        *,
        node,
        runtime,
        stop_event,
        origin_sample_count: int,
        software_radius_m: float,
        waypoint_acceptance_m: float,
        waypoint_timeout_sec: float,
        setpoint_hz: float,
        local_position_timeout_sec: float,
        yaw_speed_deg_s: float,
        yaw_acceptance_deg: float,
        yaw_timeout_sec: float,
        yaw_log_interval_sec: float = 1.0,
        log_set_yaw: bool = False,
        log_yaw_reached: bool = False,
    ):
        self.node = node
        self.logger = node.get_logger()
        self.runtime = runtime
        self.stop_event = stop_event

        self.origin_sample_count = max(int(origin_sample_count), 1)
        self.software_radius_m = float(software_radius_m)
        self.waypoint_acceptance_m = float(waypoint_acceptance_m)
        self.waypoint_timeout_sec = float(waypoint_timeout_sec)
        self.setpoint_hz = float(setpoint_hz)
        self.local_position_timeout_sec = float(local_position_timeout_sec)
        self.yaw_speed_deg_s = float(yaw_speed_deg_s)
        self.yaw_acceptance_deg = float(yaw_acceptance_deg)
        self.yaw_timeout_sec = float(yaw_timeout_sec)
        self.yaw_log_interval_sec = float(yaw_log_interval_sec)
        self.log_set_yaw = bool(log_set_yaw)
        self.log_yaw_reached = bool(log_yaw_reached)

        self.local_x = None
        self.local_y = None
        self.local_z = None
        self.last_local_position_time = 0.0
        self.current_yaw_deg = None
        self.last_attitude_time = 0.0
        self.origin_x = None
        self.origin_y = None

    def request_streams(self, local_position_hz: float = 5.0, attitude_hz: float = 10.0):
        self.runtime.mav_client.request_message_interval(
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            local_position_hz,
        )
        self.runtime.mav_client.request_message_interval(
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            attitude_hz,
        )

    def capture_origin(self, phase: str) -> bool:
        samples = []
        last_sample_time = 0.0
        start = time.monotonic()
        while time.monotonic() - start < self.local_position_timeout_sec:
            self.spin_and_drain()
            if (
                self.local_x is not None
                and self.local_position_fresh(time.monotonic())
                and self.last_local_position_time > last_sample_time
            ):
                samples.append((self.local_x, self.local_y))
                last_sample_time = self.last_local_position_time

            if len(samples) >= self.origin_sample_count:
                self.set_origin_from_samples(samples)
                self.logger.warn(
                    f"Local origin captured ({phase}): "
                    f"x={self.origin_x:.2f}, y={self.origin_y:.2f}, samples={len(samples)}"
                )
                return True

            time.sleep(0.05)

        if samples:
            self.set_origin_from_samples(samples)
            self.logger.warn(
                f"Local origin captured ({phase}, limited samples): "
                f"x={self.origin_x:.2f}, y={self.origin_y:.2f}, samples={len(samples)}"
            )
            return True

        self.logger.error(f"Failed to capture local origin ({phase}).")
        return False

    def set_origin_from_samples(self, samples):
        self.origin_x = sum(sample[0] for sample in samples) / len(samples)
        self.origin_y = sum(sample[1] for sample in samples) / len(samples)

    def goto_offset(
        self,
        label: str,
        north_m: float,
        east_m: float,
        altitude_m: float,
        acceptance_m: float = None,
        timeout_sec: float = None,
    ) -> bool:
        if self.origin_x is None or self.origin_y is None:
            self.logger.error(f"Refusing waypoint {label}: LOCAL_NED origin is unavailable.")
            return False

        if not self.target_inside_radius(north_m, east_m, self.software_radius_m):
            self.logger.error(
                f"Refusing waypoint {label}: radius={math.hypot(north_m, east_m):.2f}m "
                f"> software_radius_m={self.software_radius_m:.2f}m"
            )
            return False

        target_x = self.origin_x + north_m
        target_y = self.origin_y + east_m
        target_z = -altitude_m

        self.logger.warn(
            f"Goto {label}: north={north_m:.1f}m east={east_m:.1f}m alt={altitude_m:.1f}m"
        )

        start = time.monotonic()
        next_send = 0.0
        send_period = 1.0 / max(self.setpoint_hz, 0.1)
        acceptance_m = self.waypoint_acceptance_m if acceptance_m is None else acceptance_m
        timeout_sec = self.waypoint_timeout_sec if timeout_sec is None else timeout_sec

        while rclpy.ok() and not self.stop_event.is_set():
            now = time.monotonic()
            self.spin_and_drain()

            if not self.local_position_fresh(now):
                self.logger.error("Local position timeout during waypoint movement.")
                return False

            if self.current_radius_m() > self.software_radius_m:
                self.logger.error("Software radius breach detected during waypoint movement.")
                return False

            if now >= next_send:
                self.send_local_position_target(target_x, target_y, target_z)
                next_send = now + send_period

            distance_xy = math.hypot(self.local_x - target_x, self.local_y - target_y)
            if distance_xy <= acceptance_m:
                return True

            if now - start > timeout_sec:
                self.logger.error(f"Waypoint timeout: {label}")
                return False

            time.sleep(0.05)

        return False

    def set_yaw(self, yaw_deg: float) -> bool:
        """Ask ArduPilot to rotate toward the target yaw while staying in GUIDED mode."""

        if self.log_set_yaw:
            self.logger.info(f"Set yaw: {yaw_deg:.0f} deg")

        self.runtime.mav_client.command_long_send(
            self.runtime.mav_client.master.target_system,
            self.runtime.mav_client.master.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            yaw_deg,
            self.yaw_speed_deg_s,
            0,
            0,
            0,
            0,
            0,
        )
        return self.wait_yaw_reached(yaw_deg)

    def wait_yaw_reached(self, target_yaw_deg: float) -> bool:
        target_yaw_deg = self.normalize_angle_deg(target_yaw_deg)
        start = time.monotonic()
        last_print = 0.0

        while rclpy.ok() and not self.stop_event.is_set():
            now = time.monotonic()
            self.spin_and_drain()

            if self.yaw_fresh(now):
                yaw_error_deg = abs(self.angle_diff_deg(target_yaw_deg, self.current_yaw_deg))
                if now - last_print > self.yaw_log_interval_sec:
                    self.logger.info(
                        f"Yaw wait: current={self.current_yaw_deg:.1f} deg "
                        f"target={target_yaw_deg:.1f} deg error={yaw_error_deg:.1f} deg"
                    )
                    last_print = now

                if yaw_error_deg <= self.yaw_acceptance_deg:
                    if self.log_yaw_reached:
                        self.logger.warn(
                            f"Yaw reached: current={self.current_yaw_deg:.1f} deg "
                            f"target={target_yaw_deg:.1f} deg"
                        )
                    return True

            if now - start > self.yaw_timeout_sec:
                self.logger.error(
                    f"Yaw timeout: target={target_yaw_deg:.1f} deg current={self.current_yaw_deg}"
                )
                return False

            time.sleep(0.05)

        return False

    def send_local_position_target(self, x: float, y: float, z: float):
        """Send a GUIDED position target in LOCAL_NED coordinates."""

        self.runtime.mav_client.set_position_target_local_ned_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            self.runtime.mav_client.master.target_system,
            self.runtime.mav_client.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            POSITION_TARGET_TYPEMASK,
            x,
            y,
            z,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    def spin_and_drain(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.drain_messages()

    def drain_messages(self):
        self.runtime.mav_client.drain_messages(self.process_message)

    def process_message(self, msg):
        self.runtime.readiness.process_message(msg)

        if msg.get_type() == "LOCAL_POSITION_NED":
            self.local_x = float(msg.x)
            self.local_y = float(msg.y)
            self.local_z = float(msg.z)
            self.last_local_position_time = time.monotonic()
        elif msg.get_type() == "ATTITUDE":
            self.current_yaw_deg = self.normalize_angle_deg(math.degrees(float(msg.yaw)))
            self.last_attitude_time = time.monotonic()

    def current_offset(self):
        if self.local_x is None or self.origin_x is None:
            return None
        return self.local_x - self.origin_x, self.local_y - self.origin_y

    def current_radius_m(self) -> float:
        offset = self.current_offset()
        if offset is None:
            return 0.0
        return math.hypot(*offset)

    def current_altitude_m(self) -> float:
        if self.local_z is None:
            return 0.0
        return -self.local_z

    def local_position_fresh(self, now: float) -> bool:
        return (
            self.local_x is not None
            and now - self.last_local_position_time <= self.local_position_timeout_sec
        )

    def yaw_fresh(self, now: float) -> bool:
        return (
            self.current_yaw_deg is not None
            and now - self.last_attitude_time <= self.local_position_timeout_sec
        )

    @staticmethod
    def body_to_local_offset(forward_m: float, right_m: float, yaw_deg: float):
        yaw_rad = math.radians(yaw_deg)
        north_m = forward_m * math.cos(yaw_rad) - right_m * math.sin(yaw_rad)
        east_m = forward_m * math.sin(yaw_rad) + right_m * math.cos(yaw_rad)
        return north_m, east_m

    @staticmethod
    def target_inside_radius(north_m: float, east_m: float, radius_m: float) -> bool:
        return math.hypot(north_m, east_m) <= radius_m

    @staticmethod
    def normalize_angle_deg(angle_deg: float) -> float:
        return angle_deg % 360.0

    @staticmethod
    def angle_diff_deg(target_deg: float, current_deg: float) -> float:
        return (target_deg - current_deg + 180.0) % 360.0 - 180.0

    @staticmethod
    def yaw_to_target(delta_north_m: float, delta_east_m: float) -> float:
        return math.degrees(math.atan2(delta_east_m, delta_north_m)) % 360.0
