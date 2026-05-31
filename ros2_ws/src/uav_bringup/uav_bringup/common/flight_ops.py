#!/usr/bin/env python3

import time

from pymavlink import mavutil


class FlightOps:
    """Reusable mission actions on top of a connected MavlinkClient."""

    def __init__(self, mav_client, logger, timeout_sec: float, drain_fn):
        self.mav_client = mav_client
        self.logger = logger
        self.timeout_sec = timeout_sec
        self.drain_fn = drain_fn

    def set_mode(self, mode_name: str) -> bool:

        # Mode availability check
        mapping = self.mav_client.master.mode_mapping()

        if mapping is None or mode_name not in mapping:
            self.logger.error(f"Unsupported mode: {mode_name}")
            self.logger.error(f"Available modes: {mapping}")
            return False

        # Get ID
        mode_id = mapping[mode_name]
        self.logger.warn(f"Setting mode: {mode_name}")

        # Send desired mode
        self.mav_client.set_mode_send(
            self.mav_client.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )

        # Timeout
        start = time.time()

        while time.time() - start < self.timeout_sec:
            self.drain_fn()
            msg = self.mav_client.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            if msg is None:
                continue

            # Check current mode
            current_mode = mavutil.mode_string_v10(msg)
            self.logger.info(f"Current mode: {current_mode}")

            if current_mode.upper() == mode_name.upper():
                return True

        return False

    def arm(self) -> bool:
        self.logger.warn("Sending ARM command...")

        self.mav_client.command_long_send(
            self.mav_client.master.target_system,
            self.mav_client.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )

        ack = self.mav_client.wait_command_ack(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            self.timeout_sec,
            self.drain_fn,
        )

        if ack is None:
            self.logger.error("No ARM ACK.")
            return False

        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self.logger.warn("ARM accepted.")
            return True

        self.logger.error(f"ARM rejected: result={ack.result}")
        return False

    def wait_armed(self) -> bool:
        start = time.time()

        while time.time() - start < self.timeout_sec:
            self.drain_fn()
            msg = self.mav_client.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            if msg is None:
                continue

            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.logger.info(f"Armed state: {armed}")

            if armed:
                return True

        return False

    def takeoff(self, altitude_m: float) -> bool:
        self.logger.warn(f"Sending TAKEOFF: {altitude_m:.2f} m")

        self.mav_client.command_long_send(
            self.mav_client.master.target_system,
            self.mav_client.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # Send TAKEOFF
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            altitude_m,
        )

        # Received ACK check
        ack = self.mav_client.wait_command_ack(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            self.timeout_sec,
            self.drain_fn,
        )

        # return True
        if ack is None:
            self.logger.warn("No TAKEOFF ACK. Continue altitude monitoring.")
            return True

        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self.logger.warn("TAKEOFF accepted.")
            return True

        # return false
        self.logger.error(f"TAKEOFF rejected: result={ack.result}")
        return False

    def wait_altitude_reached(self, altitude_m: float, altitude_ratio: float) -> bool:
        target_alt_m = altitude_m * altitude_ratio  # unit: meters
        start = time.time()

        self.logger.warn(f"Waiting relative_alt >= {target_alt_m:.2f} m")

        while time.time() - start < max(self.timeout_sec, 30.0):  # Max 30 seconds
            self.drain_fn()
            msg = self.mav_client.master.recv_match(
                type="GLOBAL_POSITION_INT",
                blocking=True,
                timeout=1.0,
            )
            if msg is None:
                continue

            relative_alt_m = float(msg.relative_alt) / 1000.0  # msg.relative_alt unit: millimeters
            self.logger.info(
                f"relative_alt={relative_alt_m:.2f} m / target={altitude_m:.2f} m"
            )

            if relative_alt_m >= target_alt_m:
                self.logger.warn("Altitude threshold reached.")
                return True

        return False
