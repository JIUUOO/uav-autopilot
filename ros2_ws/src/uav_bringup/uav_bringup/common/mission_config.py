#!/usr/bin/env python3

from dataclasses import dataclass
import os


@dataclass
class GuidedTakeoffMissionConfig:
    port: str
    baudrate: int
    dry_run: bool
    altitude_m: float
    altitude_ratio: float
    timeout_sec: float
    ready_timeout_sec: float
    loiter_hold_sec: float
    enable_ntrip: bool
    ntrip_host: str
    ntrip_port: int
    ntrip_mountpoint: str
    ntrip_user: str
    ntrip_pass: str
    min_gps_fix_type: int
    max_hacc_m: float
    require_optical_flow: bool
    optical_flow_timeout_sec: float
    land_after_hold: bool = False


def declare_guided_takeoff_params(
    node,
    include_land_after_hold: bool = False,
    default_altitude_m: float = 1.0,
):
    node.declare_parameter("port", "/dev/ttyACM0")
    node.declare_parameter("baudrate", 115200)
    node.declare_parameter("dry_run", True)

    node.declare_parameter("altitude_m", default_altitude_m)  # objective altitude (GUIDED mode)
    node.declare_parameter("altitude_ratio", 0.85)  # altitude threshold for GUIDED -> LOITER mode
    node.declare_parameter("timeout_sec", 20.0)  # common timeout for heartbeat, ACK, mode, arm waits
    node.declare_parameter("ready_timeout_sec", 90.0)  # max wait for '(GPS, RTK) + optical flow' readiness before takeoff
    node.declare_parameter("loiter_hold_sec", 0.0)  # LOITER hold seconds after takeoff

    if include_land_after_hold:
        node.declare_parameter("land_after_hold", True)

    # core of RTK F9P GNSS
    node.declare_parameter("enable_ntrip", True)  # for fix_type to be 6
    node.declare_parameter("ntrip_host", "www.gnssdata.or.kr")  # GNSS provider in Korea
    node.declare_parameter("ntrip_port", 2101)
    node.declare_parameter("ntrip_mountpoint", "SUWN-RTCM31")  # Suwon
    node.declare_parameter("ntrip_user", os.getenv("NTRIP_USER", ""))
    node.declare_parameter("ntrip_pass", os.getenv("NTRIP_PASS", "gnss"))

    node.declare_parameter("min_gps_fix_type", 4)
    node.declare_parameter("max_hacc_m", 5.0)  # ready check fails when hacc_m > max_hacc_m

    node.declare_parameter("require_optical_flow", True)
    node.declare_parameter("optical_flow_timeout_sec", 3.0)  # flow is ready only if recent flow messages exist


def load_guided_takeoff_config(node, include_land_after_hold: bool = False) -> GuidedTakeoffMissionConfig:
    land_after_hold = False
    if include_land_after_hold:
        land_after_hold = bool(node.get_parameter("land_after_hold").value)

    return GuidedTakeoffMissionConfig(
        port=str(node.get_parameter("port").value),
        baudrate=int(node.get_parameter("baudrate").value),
        dry_run=bool(node.get_parameter("dry_run").value),
        altitude_m=float(node.get_parameter("altitude_m").value),
        altitude_ratio=float(node.get_parameter("altitude_ratio").value),
        timeout_sec=float(node.get_parameter("timeout_sec").value),
        ready_timeout_sec=float(node.get_parameter("ready_timeout_sec").value),
        loiter_hold_sec=float(node.get_parameter("loiter_hold_sec").value),
        enable_ntrip=bool(node.get_parameter("enable_ntrip").value),
        ntrip_host=str(node.get_parameter("ntrip_host").value),
        ntrip_port=int(node.get_parameter("ntrip_port").value),
        ntrip_mountpoint=str(node.get_parameter("ntrip_mountpoint").value),
        ntrip_user=str(node.get_parameter("ntrip_user").value),
        ntrip_pass=str(node.get_parameter("ntrip_pass").value),
        min_gps_fix_type=int(node.get_parameter("min_gps_fix_type").value),
        max_hacc_m=float(node.get_parameter("max_hacc_m").value),
        require_optical_flow=bool(node.get_parameter("require_optical_flow").value),
        optical_flow_timeout_sec=float(node.get_parameter("optical_flow_timeout_sec").value),
        land_after_hold=land_after_hold,
    )
