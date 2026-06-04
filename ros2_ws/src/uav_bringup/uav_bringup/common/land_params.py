#!/usr/bin/env python3

from dataclasses import dataclass


@dataclass
class LandSpeedParams:
    set_land_speed_params: bool
    land_speed_cm_s: float
    land_speed_high_cm_s: float
    land_param_timeout_sec: float


def declare_land_speed_params(node):
    node.declare_parameter("set_land_speed_params", True)  # Whether to set Pixhawk LAND speed parameters before arm/takeoff.
    node.declare_parameter("land_speed_cm_s", 20.0)  # Final LAND descent speed near the ground (cm/s).
    node.declare_parameter("land_speed_high_cm_s", 20.0)  # Initial/high-altitude LAND descent speed (cm/s).
    node.declare_parameter("land_param_timeout_sec", 5.0)  # Max wait time for Pixhawk parameter confirmation (seconds).


def load_land_speed_params(node) -> LandSpeedParams:
    return LandSpeedParams(
        set_land_speed_params=bool(node.get_parameter("set_land_speed_params").value),
        land_speed_cm_s=float(node.get_parameter("land_speed_cm_s").value),
        land_speed_high_cm_s=float(node.get_parameter("land_speed_high_cm_s").value),
        land_param_timeout_sec=float(node.get_parameter("land_param_timeout_sec").value),
    )


def configure_land_speed_params(node, mav_client, params: LandSpeedParams) -> bool:
    if not params.set_land_speed_params:
        node.get_logger().warn("set_land_speed_params=false. Keeping existing FC LAND speed parameters.")
        return True

    node.get_logger().warn(
        "Configuring LAND descent speed before arm/takeoff: "
        f"LAND_SPEED={params.land_speed_cm_s:.0f}cm/s, "
        f"LAND_SPEED_HIGH={params.land_speed_high_cm_s:.0f}cm/s"
    )

    fc_params = [
        ("LAND_SPEED", params.land_speed_cm_s),
        ("LAND_SPEED_HIGH", params.land_speed_high_cm_s),
    ]
    for name, value in fc_params:
        if not mav_client.set_param_float(
            name,
            value,
            params.land_param_timeout_sec,
        ):
            return False

    return True
