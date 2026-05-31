#!/usr/bin/env python3

import time


FIX_NAME = {
    0: "NO_GPS",
    1: "NO_FIX",
    2: "2D_FIX",
    3: "3D_FIX",
    4: "DGPS",
    5: "RTK_FLOAT",
    6: "RTK_FIXED",
}


class ReadinessMonitor:
    """Tracks GPS/flow freshness and evaluates preflight readiness."""

    def __init__(
        self,
        logger,
        min_gps_fix_type: int,
        max_hacc_m: float,
        require_optical_flow: bool,
        optical_flow_timeout_sec: float,
        ready_timeout_sec: float,
    ):
        self.logger = logger
        self.min_gps_fix_type = min_gps_fix_type
        self.max_hacc_m = max_hacc_m
        self.require_optical_flow = require_optical_flow
        self.optical_flow_timeout_sec = optical_flow_timeout_sec
        self.ready_timeout_sec = ready_timeout_sec

        self.gps_fix_type = 0
        self.gps_sats = 0
        self.gps_hacc_mm = None
        self.gps_vacc_mm = None
        self.last_gps_time = 0.0
        self.last_flow_time = 0.0
        self.last_rangefinder_time = 0.0

    def process_message(self, msg):
        msg_type = msg.get_type()
        now = time.time()

        if msg_type == "GPS_RAW_INT":
            self.gps_fix_type = int(msg.fix_type)
            self.gps_sats = int(msg.satellites_visible)
            self.gps_hacc_mm = getattr(msg, "h_acc", None)
            self.gps_vacc_mm = getattr(msg, "v_acc", None)
            self.last_gps_time = now
        elif msg_type in ["OPTICAL_FLOW", "OPTICAL_FLOW_RAD"]:
            self.last_flow_time = now
        elif msg_type == "DISTANCE_SENSOR":
            self.last_rangefinder_time = now

    def gps_ready(self) -> bool:
        if time.time() - self.last_gps_time > 3.0:
            return False
        if self.gps_fix_type < self.min_gps_fix_type:
            return False

        if self.gps_hacc_mm is not None:
            hacc_m = float(self.gps_hacc_mm) / 1000.0
            if hacc_m > self.max_hacc_m:
                return False

        return True

    def flow_ready(self) -> bool:
        if not self.require_optical_flow:
            return True
        return (time.time() - self.last_flow_time) <= self.optical_flow_timeout_sec

    def format_status(self, prefix: str, ntrip_connected: bool, rtcm_frames: int, now: float):
        fix_name = FIX_NAME.get(self.gps_fix_type, "UNKNOWN")
        hacc_m = None if self.gps_hacc_mm is None else self.gps_hacc_mm / 1000.0
        flow_age = None if self.last_flow_time <= 0 else now - self.last_flow_time
        self.logger.info(
            f"{prefix}: "
            f"ntrip={ntrip_connected} "
            f"rtcm_frames={rtcm_frames} "
            f"fix={self.gps_fix_type}({fix_name}) "
            f"sats={self.gps_sats} "
            f"hacc_m={hacc_m} "
            f"flow_age={flow_age}"
        )

    def wait_ready(self, drain_fn, stop_event, ntrip_connected_fn, rtcm_frames_fn) -> bool:
        self.logger.warn("Waiting GPS/DGPS/RTK and Optical Flow readiness...")

        start = time.time()
        last_print = 0.0

        while time.time() - start < self.ready_timeout_sec:
            drain_fn()

            now = time.time()
            if now - last_print > 2.0:
                self.format_status(
                    "ready_check",
                    ntrip_connected=ntrip_connected_fn(),
                    rtcm_frames=rtcm_frames_fn(),
                    now=now,
                )
                last_print = now

            if self.gps_ready() and self.flow_ready():
                self.logger.warn("Preflight readiness OK.")
                return True

            time.sleep(0.05)

        return False

