#!/usr/bin/env python3

from uav_bringup.common.flight_ops import FlightOps
from uav_bringup.common.mavlink_client import MavlinkClient
from uav_bringup.common.ntrip import NtripForwarder
from uav_bringup.common.readiness import ReadinessMonitor


class MissionRuntime:
    """Shared mission runtime: MAVLink, readiness checks, and optional NTRIP forwarding."""

    def __init__(self, node, config, stop_event):
        self.node = node
        self.logger = node.get_logger()
        self.config = config
        self.stop_event = stop_event

        self.mav_client = MavlinkClient(
            port=self.config.port,
            baudrate=self.config.baudrate,
            timeout_sec=self.config.timeout_sec,
            logger=self.logger,
        )
        self.readiness = ReadinessMonitor(
            logger=self.logger,
            min_gps_fix_type=self.config.min_gps_fix_type,
            max_hacc_m=self.config.max_hacc_m,
            require_optical_flow=self.config.require_optical_flow,
            optical_flow_timeout_sec=self.config.optical_flow_timeout_sec,
            ready_timeout_sec=self.config.ready_timeout_sec,
        )

        self.ntrip_forwarder = None

    def log_config(self, mission_name: str, include_land_after_hold: bool = False):
        self.logger.warn(mission_name)
        self.logger.info(f"port                 : {self.config.port}")
        self.logger.info(f"baudrate             : {self.config.baudrate}")
        self.logger.info(f"dry_run              : {self.config.dry_run}")
        self.logger.info(f"altitude_m           : {self.config.altitude_m}")
        self.logger.info(f"enable_ntrip         : {self.config.enable_ntrip}")
        self.logger.info(f"ntrip_host           : {self.config.ntrip_host}")
        self.logger.info(f"ntrip_mountpoint     : {self.config.ntrip_mountpoint}")
        self.logger.info(f"min_gps_fix_type     : {self.config.min_gps_fix_type}")
        self.logger.info(f"max_hacc_m           : {self.config.max_hacc_m}")
        self.logger.info(f"require_optical_flow : {self.config.require_optical_flow}")
        self.logger.info(f"require_battery_check: {self.config.require_battery_check}")
        self.logger.info(f"min_battery_voltage_v: {self.config.min_battery_voltage_v}")
        self.logger.info(f"battery_check_timeout : {self.config.battery_check_timeout_sec}")
        self.logger.info(f"loiter_hold_sec      : {self.config.loiter_hold_sec}")
        if include_land_after_hold:
            self.logger.info(f"land_after_hold      : {self.config.land_after_hold}")

    def connect_and_prepare(self, stream_hz: float = 5.0, include_flow_rad: bool = True) -> bool:
        self.mav_client.connect()
        self.mav_client.request_default_streams(hz=stream_hz, include_flow_rad=include_flow_rad)

        if not self.check_battery_before_mission(stage="initial"):
            return False

        if not self.start_ntrip_if_enabled():
            return False

        if not self.wait_readiness():
            self.logger.error("Readiness check failed.")
            return False

        if not self.check_battery_before_mission(stage="final"):
            return False

        return True

    def check_battery_before_mission(self, stage: str = "preflight") -> bool:
        """Block the mission before NTRIP/arm/takeoff when battery voltage is unsafe."""

        if not self.config.require_battery_check:
            self.logger.warn("Preflight battery check disabled.")
            return True

        self.logger.warn(
            f"Waiting for {stage} battery voltage: "
            f"must be > {self.config.min_battery_voltage_v:.2f} V"
        )
        voltage_v = self.mav_client.wait_battery_voltage(
            timeout_sec=self.config.battery_check_timeout_sec
        )
        if voltage_v is None:
            self.logger.error(
                f"MISSION BLOCKED ({stage}): no valid SYS_STATUS battery voltage received "
                f"within {self.config.battery_check_timeout_sec:.1f}s."
            )
            return False

        if voltage_v <= self.config.min_battery_voltage_v:
            self.logger.error(
                f"MISSION BLOCKED ({stage}): battery voltage "
                f"{voltage_v:.2f} V <= {self.config.min_battery_voltage_v:.2f} V."
            )
            return False

        self.logger.warn(f"Battery OK ({stage}): {voltage_v:.2f} V")
        return True

    def start_ntrip_if_enabled(self) -> bool:
        if not self.config.enable_ntrip:
            return True

        if not self.config.ntrip_user:
            self.logger.error("NTRIP user empty. Set NTRIP_USER or -p ntrip_user:=...")
            return False

        self.ntrip_forwarder = NtripForwarder(
            mav_client=self.mav_client,
            logger=self.logger,
            stop_event=self.stop_event,
            host=self.config.ntrip_host,
            port=self.config.ntrip_port,
            mountpoint=self.config.ntrip_mountpoint,
            user=self.config.ntrip_user,
            password=self.config.ntrip_pass,
        )
        self.ntrip_forwarder.start()
        return True

    def wait_readiness(self) -> bool:
        return self.readiness.wait_ready(
            drain_fn=self.drain_mavlink,
            stop_event=self.stop_event,
            ntrip_connected_fn=self.is_ntrip_connected,
            rtcm_frames_fn=self.get_rtcm_frames,
        )

    def make_flight_ops(self) -> FlightOps:
        return FlightOps(
            mav_client=self.mav_client,
            logger=self.logger,
            timeout_sec=self.config.timeout_sec,
            drain_fn=self.drain_mavlink,
        )

    def drain_mavlink(self):
        """Drain pending MAVLink messages (queue) and update readiness state from incoming telemetry."""
        self.mav_client.drain_messages(self.readiness.process_message)

    def is_ntrip_connected(self) -> bool:
        if self.ntrip_forwarder is None:
            return False
        return self.ntrip_forwarder.connected

    def get_rtcm_frames(self) -> int:
        if self.ntrip_forwarder is None:
            return 0
        return self.ntrip_forwarder.frame_count

    def should_abort(self) -> bool:
        return self.stop_event.is_set()

    def stop_if_dry_run(self, message: str) -> bool:
        if not self.config.dry_run:
            return False

        self.logger.warn(message)
        self.finish()
        return True

    def finish(self):
        self.stop_event.set()
