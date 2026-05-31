#!/usr/bin/env python3

import threading
import time

from pymavlink import mavutil


class MavlinkClient:
    """Thin wrapper around pymavlink connection and command I/O."""

    def __init__(self, port: str, baudrate: int, timeout_sec: float, logger):
        self.port = port
        self.baudrate = baudrate
        self.timeout_sec = timeout_sec
        self.logger = logger

        self.master = None
        self.send_lock = threading.Lock()

    def connect(self):
        self.master = mavutil.mavlink_connection(
            self.port,
            baud=self.baudrate,
            autoreconnect=True,
        )

        self.logger.info("Waiting heartbeat...")
        self.master.wait_heartbeat(timeout=self.timeout_sec)
        self.logger.info(
            f"Heartbeat OK: system={self.master.target_system}, component={self.master.target_component}"
        )

    def request_message_interval(self, msg_id: int, hz: float):
        interval_us = int(1_000_000 / hz)

        self.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )

    def request_default_streams(self, hz: float = 5.0, include_flow_rad: bool = True):
        msg_ids = [
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            mavutil.mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW,
            mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
            mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
        ]

        if include_flow_rad and hasattr(mavutil.mavlink, "MAVLINK_MSG_ID_OPTICAL_FLOW_RAD"):
            msg_ids.append(mavutil.mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW_RAD)

        for msg_id in msg_ids:
            self.request_message_interval(msg_id, hz)

    def command_long_send(self, *args):
        with self.send_lock:
            self.master.mav.command_long_send(*args)

    def set_mode_send(self, *args):
        with self.send_lock:
            self.master.mav.set_mode_send(*args)

    def drain_messages(self, handler):
        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                return
            handler(msg)

    def wait_command_ack(self, command_id: int, timeout_sec: float, drain_fn):
        start = time.time()

        while time.time() - start < timeout_sec:
            drain_fn()
            msg = self.master.recv_match(type="COMMAND_ACK", blocking=True, timeout=1.0)
            if msg is None:
                continue
            if int(msg.command) == int(command_id):
                return msg

        return None
