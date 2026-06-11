#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from rclpy.node import Node
from uav_interfaces.msg import GeminiReport

from uav_bringup.common.local_ned_controller import LocalNedController
from uav_bringup.common.mission_config import declare_guided_takeoff_params
from uav_bringup.common.mission_config import load_guided_takeoff_config
from uav_bringup.common.mission_runtime import MissionRuntime


"""
Mission flow:
1. Connect MAVLink, request telemetry streams, and wait for readiness.
2. Capture LOCAL_NED origin before takeoff so CENTER matches the ground start point.
3. Switch to GUIDED, arm, and take off to scout altitude.
4. Fly the bounded rule-based route: CENTER -> N -> NE -> E -> SE -> S -> SW -> W -> NW -> CENTER.
5. At each waypoint, run yaw sweep and hold while watching Gemini reports.
6. If Gemini requests inspection, optionally descend within safety limits, hold, then climb back.
7. Return to CENTER (captured LOCAL_NED origin, not Pixhawk Home) and switch to finish_mode, normally LAND.
"""

# LOCAL_NED uses x=North, y=East, z=Down; altitude above home is negative z.

class BoundedScoutMissionNode(Node):
    """
    - Fixed cardinal/corner scout route inside a software radius.
    - Gemini HIGH/inspect reports can trigger optional low-altitude inspection.
    - Pixhawk geofence should still be configured as the hard safety boundary.
    """

    def __init__(self):
        super().__init__("bounded_scout_mission_node")

        declare_guided_takeoff_params(self, include_land_after_hold=True, default_altitude_m=3.5)
        self.declare_scout_params()

        self.config = load_guided_takeoff_config(self, include_land_after_hold=True)
        self.load_scout_params()

        self.stop_event = threading.Event()
        self.runtime = MissionRuntime(node=self, config=self.config, stop_event=self.stop_event)
        self.flight_ops = None

        self.local_ned = LocalNedController(
            node=self,
            runtime=self.runtime,
            stop_event=self.stop_event,
            origin_sample_count=self.origin_sample_count,
            software_radius_m=self.software_radius_m,
            waypoint_acceptance_m=self.waypoint_acceptance_m,
            waypoint_timeout_sec=self.waypoint_timeout_sec,
            setpoint_hz=self.setpoint_hz,
            local_position_timeout_sec=self.local_position_timeout_sec,
            yaw_speed_deg_s=self.yaw_speed_deg_s,
            yaw_acceptance_deg=self.yaw_acceptance_deg,
            yaw_timeout_sec=self.yaw_timeout_sec,
            yaw_log_interval_sec=0.5,
            log_set_yaw=True,
            log_yaw_reached=True,
        )

        self.latest_report = None
        self.latest_report_time = 0.0
        self.latest_report_consumed = True
        self.inspection_count = 0

        self.create_subscription(
            GeminiReport,
            self.gemini_report_topic,
            self.on_gemini_report,
            10,
        )

        self.run_once()

    def declare_scout_params(self):
        # Route shape and software boundary limits.
        self.declare_parameter("scout_radius_m", 5.0)
        self.declare_parameter("corner_offset_m", 5.0)
        self.declare_parameter("software_radius_m", 8.0)
        self.declare_parameter("inspection_radius_m", 7.2)
        self.declare_parameter("route_pattern", "N,NE,E,SE,S,SW,W,NW")

        # LOCAL_NED reference and waypoint movement control.
        self.declare_parameter("require_origin_before_takeoff", True)
        self.declare_parameter("origin_sample_count", 5)  # LOCAL_NED samples averaged for this node's CENTER, route, and radius checks.
        self.declare_parameter("waypoint_acceptance_m", 0.8)
        self.declare_parameter("waypoint_timeout_sec", 30.0)
        self.declare_parameter("setpoint_hz", 2.0)  # Position setpoint send rate to Pixhawk (Hz).
        self.declare_parameter("local_position_timeout_sec", 5.0)

        # Per-waypoint yaw scan and hold timing.
        self.declare_parameter("waypoint_hold_sec", 5.0)
        self.declare_parameter("yaw_angles_deg", "0,90,180,270")
        self.declare_parameter("yaw_hold_sec", 2.0)
        self.declare_parameter("yaw_speed_deg_s", 20.0)  # (deg/s)
        self.declare_parameter("yaw_acceptance_deg", 8.0)
        self.declare_parameter("yaw_timeout_sec", 8.0)
        self.declare_parameter("route_mode", "waypoints")
        self.declare_parameter("square_side_m", 2.0)
        self.declare_parameter("square_yaws_deg", "0,90,180,270")
        self.declare_parameter("square_origin_mode", "center")

        # Gemini-triggered inspection behavior and safety gates.
        self.declare_parameter("gemini_report_topic", "/uav/vision/gemini_report")
        self.declare_parameter("inspection_confidence_threshold", 0.70)
        self.declare_parameter("enable_low_altitude_inspection", False)
        self.declare_parameter("inspect_altitude_m", 2.0)
        self.declare_parameter("min_safe_altitude_m", 2.0)
        self.declare_parameter("inspection_hold_sec", 5.0)
        self.declare_parameter("inspection_max_count", 1)
        self.declare_parameter("disable_inspection_for_person", True)
        self.declare_parameter("person_safe_altitude_m", 3.5)

        # Mission exit behavior.
        self.declare_parameter("finish_mode", "LAND")
        self.declare_parameter("abort_mode", "RTL")  # Return to Launch

    def load_scout_params(self):
        self.scout_radius_m = float(self.get_parameter("scout_radius_m").value)
        self.corner_offset_m = float(self.get_parameter("corner_offset_m").value)
        self.software_radius_m = float(self.get_parameter("software_radius_m").value)
        self.inspection_radius_m = float(self.get_parameter("inspection_radius_m").value)
        self.require_origin_before_takeoff = bool(self.get_parameter("require_origin_before_takeoff").value)
        self.origin_sample_count = max(int(self.get_parameter("origin_sample_count").value), 1)
        self.waypoint_acceptance_m = float(self.get_parameter("waypoint_acceptance_m").value)
        self.waypoint_timeout_sec = float(self.get_parameter("waypoint_timeout_sec").value)
        self.waypoint_hold_sec = float(self.get_parameter("waypoint_hold_sec").value)
        self.route_pattern = str(self.get_parameter("route_pattern").value)
        self.yaw_angles_deg = self.parse_float_list(str(self.get_parameter("yaw_angles_deg").value))
        self.yaw_hold_sec = float(self.get_parameter("yaw_hold_sec").value)
        self.yaw_speed_deg_s = float(self.get_parameter("yaw_speed_deg_s").value)
        self.yaw_acceptance_deg = float(self.get_parameter("yaw_acceptance_deg").value)
        self.yaw_timeout_sec = float(self.get_parameter("yaw_timeout_sec").value)
        self.setpoint_hz = float(self.get_parameter("setpoint_hz").value)
        self.local_position_timeout_sec = float(self.get_parameter("local_position_timeout_sec").value)
        self.route_mode = str(self.get_parameter("route_mode").value).strip().lower()
        self.square_side_m = float(self.get_parameter("square_side_m").value)
        self.square_yaws_deg = self.parse_float_list(str(self.get_parameter("square_yaws_deg").value))
        self.square_origin_mode = str(self.get_parameter("square_origin_mode").value).strip().lower()

        self.gemini_report_topic = str(self.get_parameter("gemini_report_topic").value)
        self.inspection_confidence_threshold = float(self.get_parameter("inspection_confidence_threshold").value)
        self.enable_low_altitude_inspection = bool(self.get_parameter("enable_low_altitude_inspection").value)
        self.inspect_altitude_m = float(self.get_parameter("inspect_altitude_m").value)
        self.min_safe_altitude_m = float(self.get_parameter("min_safe_altitude_m").value)
        self.inspection_hold_sec = float(self.get_parameter("inspection_hold_sec").value)
        self.inspection_max_count = int(self.get_parameter("inspection_max_count").value)
        self.disable_inspection_for_person = bool(self.get_parameter("disable_inspection_for_person").value)
        self.person_safe_altitude_m = float(self.get_parameter("person_safe_altitude_m").value)

        self.finish_mode = str(self.get_parameter("finish_mode").value).upper()
        self.abort_mode = str(self.get_parameter("abort_mode").value).upper()

    def run_once(self):
        self.runtime.log_config("=== Bounded Scout Mission ===", include_land_after_hold=True)
        self.log_scout_config()

        if not self.runtime.connect_and_prepare(stream_hz=5.0, include_flow_rad=True):
            self.runtime.finish()
            return

        self.request_scout_streams()
        self.flight_ops = self.runtime.make_flight_ops()

        if self.runtime.stop_if_dry_run("Dry-run enabled. Scout movement/arm/takeoff commands not sent."):
            return

        origin_captured = self.capture_local_origin("pre_takeoff")
        if not origin_captured and self.require_origin_before_takeoff:
            self.abort_mission("pre_takeoff_origin_failed", set_abort_mode=False)
            return

        if not self.takeoff_to_scout_altitude():
            self.abort_mission("takeoff_failed")
            return

        if not origin_captured and not self.capture_local_origin("post_takeoff"):
            self.abort_mission("local_origin_failed")
            return

        if not self.run_scout_route():
            self.abort_mission("scout_route_failed")
            return

        if not self.goto_offset("CENTER", 0.0, 0.0, self.config.altitude_m):
            self.abort_mission("return_center_failed")
            return

        self.finish_mission()

    def takeoff_to_scout_altitude(self) -> bool:
        if not self.flight_ops.set_mode("GUIDED"):
            return False
        if not self.flight_ops.arm():
            return False
        if not self.flight_ops.wait_armed():
            return False
        if not self.flight_ops.takeoff(self.config.altitude_m):
            return False
        return self.flight_ops.wait_altitude_reached(
            self.config.altitude_m,
            self.config.altitude_ratio,
        )

    def run_scout_route(self) -> bool:
        if self.route_mode == "forward_square":
            return self.run_forward_square_route()

        for name, north_m, east_m in self.route_offsets():
            if not self.goto_offset(name, north_m, east_m, self.config.altitude_m):
                return False

            if not self.run_yaw_sweep_and_hold(name):
                return False

            if self.should_run_inspection():
                self.latest_report_consumed = True
                if not self.run_low_altitude_inspection(name, north_m, east_m):
                    return False

        return True

    def run_forward_square_route(self) -> bool:
        if self.square_origin_mode == "center":
            return self.run_centered_forward_square_route()
        if self.square_origin_mode != "corner":
            self.get_logger().error("square_origin_mode must be either 'center' or 'corner'.")
            return False

        if len(self.square_yaws_deg) != 4:
            self.get_logger().error("square_yaws_deg must contain exactly four yaw angles in corner mode.")
            return False

        self.get_logger().warn(
            "Forward square corner mode: CENTER is treated as one square corner, not the square center."
        )
        north_m = 0.0
        east_m = 0.0
        for index, yaw_deg in enumerate(self.square_yaws_deg, start=1):
            label = f"LEG{index}/yaw={yaw_deg:.0f}"
            if not self.set_yaw(yaw_deg):
                return False
            if not self.hold_and_watch_reports(label=f"{label}/yaw_settle", hold_sec=self.yaw_hold_sec):
                return False

            yaw_rad = math.radians(yaw_deg)
            north_m += math.cos(yaw_rad) * self.square_side_m
            east_m += math.sin(yaw_rad) * self.square_side_m

            if not self.goto_offset(f"FWD{index}", north_m, east_m, self.config.altitude_m):
                return False
            if not self.hold_and_watch_reports(label=f"FWD{index}/analysis_hold", hold_sec=self.waypoint_hold_sec):
                return False

            if self.should_run_inspection():
                self.latest_report_consumed = True
                if not self.run_low_altitude_inspection(f"FWD{index}", north_m, east_m):
                    return False

        return True

    def run_centered_forward_square_route(self) -> bool:
        half_side_m = self.square_side_m / 2.0
        points = [
            ("CORNER_NW", half_side_m, -half_side_m),
            ("CORNER_NE", half_side_m, half_side_m),
            ("CORNER_SE", -half_side_m, half_side_m),
            ("CORNER_SW", -half_side_m, -half_side_m),
            ("CORNER_NW_RETURN", half_side_m, -half_side_m),
            ("CENTER_RETURN", 0.0, 0.0),
        ]

        current_north_m = 0.0
        current_east_m = 0.0
        for label, target_north_m, target_east_m in points:
            yaw_deg = self.yaw_to_target(
                target_north_m - current_north_m,
                target_east_m - current_east_m,
            )
            if not self.set_yaw(yaw_deg):
                return False
            if not self.hold_and_watch_reports(label=f"{label}/yaw_settle", hold_sec=self.yaw_hold_sec):
                return False

            if not self.goto_offset(label, target_north_m, target_east_m, self.config.altitude_m):
                return False
            if not self.hold_and_watch_reports(label=f"{label}/analysis_hold", hold_sec=self.waypoint_hold_sec):
                return False

            if self.should_run_inspection():
                self.latest_report_consumed = True
                if not self.run_low_altitude_inspection(label, target_north_m, target_east_m):
                    return False

            current_north_m = target_north_m
            current_east_m = target_east_m

        return True

    def run_yaw_sweep_and_hold(self, waypoint_name: str) -> bool:
        for yaw_deg in self.yaw_angles_deg:
            if not self.set_yaw(yaw_deg):
                return False
            if not self.hold_and_watch_reports(
                label=f"{waypoint_name}/yaw={yaw_deg:.0f}",
                hold_sec=self.yaw_hold_sec,
            ):
                return False

        return self.hold_and_watch_reports(
            label=f"{waypoint_name}/analysis_hold",
            hold_sec=self.waypoint_hold_sec,
        )

    def run_low_altitude_inspection(self, waypoint_name: str, north_m: float, east_m: float) -> bool:
        if not self.enable_low_altitude_inspection:
            self.get_logger().warn(f"Inspection trigger at {waypoint_name}, but low-altitude inspection is disabled.")
            return True

        if self.inspection_count >= self.inspection_max_count:
            self.get_logger().warn("Inspection trigger ignored: inspection_max_count reached.")
            return True

        if math.hypot(north_m, east_m) > self.inspection_radius_m:
            self.get_logger().warn("Inspection trigger ignored: waypoint outside inspection radius.")
            return True

        inspect_altitude_m = max(self.inspect_altitude_m, self.min_safe_altitude_m)
        self.inspection_count += 1
        self.get_logger().warn(
            f"Starting low-altitude inspection #{self.inspection_count} at {waypoint_name}: "
            f"{inspect_altitude_m:.1f}m"
        )

        if not self.goto_offset(f"{waypoint_name}/INSPECT_DESCEND", north_m, east_m, inspect_altitude_m):
            return False
        if not self.hold_and_watch_reports(
            label=f"{waypoint_name}/INSPECT_HOLD",
            hold_sec=self.inspection_hold_sec,
            allow_inspection_trigger=False,
        ):
            return False
        return self.goto_offset(f"{waypoint_name}/INSPECT_ASCEND", north_m, east_m, self.config.altitude_m)

    def goto_offset(self, label: str, north_m: float, east_m: float, altitude_m: float) -> bool:
        return self.local_ned.goto_offset(label, north_m, east_m, altitude_m)

    def hold_and_watch_reports(
        self,
        *,
        label: str,
        hold_sec: float,
        allow_inspection_trigger: bool = True,
    ) -> bool:
        start = time.monotonic()
        last_print = 0.0

        while rclpy.ok() and not self.stop_event.is_set():
            now = time.monotonic()
            self.spin_and_drain()

            if not self.local_position_fresh(now):
                self.get_logger().error("Local position timeout during hold.")
                return False

            if self.current_radius_m() > self.software_radius_m:
                self.get_logger().error("Software radius breach detected during hold.")
                return False

            if now - last_print > 2.0:
                self.get_logger().info(
                    f"Hold {label}: radius={self.current_radius_m():.2f}m "
                    f"alt={self.current_altitude_m():.2f}m"
                )
                last_print = now

            if allow_inspection_trigger and self.should_run_inspection():
                return True

            if now - start >= hold_sec:
                return True

            time.sleep(0.05)

        return False

    def should_run_inspection(self) -> bool:
        if self.latest_report is None or self.latest_report_consumed:
            return False

        report = self.latest_report
        if not report.parsed_ok:
            self.latest_report_consumed = True
            return False

        candidate = self.find_candidate(
            report.target_candidates,
            report.primary_candidate_index,
        )
        confidence_ok = (
            candidate is not None
            and candidate.confidence >= self.inspection_confidence_threshold
        )

        if report.target_detected and confidence_ok:
            if self.disable_inspection_for_person and self.is_person_candidate(candidate):
                self.get_logger().warn(
                    "Inspection blocked: person detected in Gemini report. "
                    f"Keeping scout altitude >= {self.person_safe_altitude_m:.1f}m."
                )
                self.latest_report_consumed = True
                return False

            self.get_logger().warn(
                f"Inspection trigger: candidate={candidate.candidate_index} "
                f"confidence={candidate.confidence:.2f}"
            )
            return True

        self.latest_report_consumed = True
        return False

    def set_yaw(self, yaw_deg: float) -> bool:
        """Ask ArduPilot to rotate toward the target yaw while staying in GUIDED mode."""

        return self.local_ned.set_yaw(yaw_deg)

    @staticmethod
    def yaw_to_target(delta_north_m: float, delta_east_m: float) -> float:
        return math.degrees(math.atan2(delta_east_m, delta_north_m)) % 360.0

    def request_scout_streams(self):
        self.local_ned.request_streams(local_position_hz=5.0, attitude_hz=10.0)

    def capture_local_origin(self, phase: str) -> bool:
        return self.local_ned.capture_origin(phase)

    def on_gemini_report(self, msg):
        self.latest_report = msg
        self.latest_report_time = time.monotonic()
        self.latest_report_consumed = False

    def spin_and_drain(self):
        self.local_ned.spin_and_drain()

    def finish_mission(self):
        if self.finish_mode:
            self.flight_ops.set_mode(self.finish_mode)
        self.runtime.finish()

    def abort_mission(self, reason: str, set_abort_mode: bool = True):
        self.get_logger().error(f"Mission abort: {reason}")
        if set_abort_mode and self.flight_ops is not None and self.abort_mode:
            self.flight_ops.set_mode(self.abort_mode)
        self.runtime.finish()

    def log_scout_config(self):
        self.get_logger().info(f"scout_radius_m                 : {self.scout_radius_m}")
        self.get_logger().info(f"corner_offset_m                : {self.corner_offset_m}")
        self.get_logger().info(f"software_radius_m              : {self.software_radius_m}")
        self.get_logger().info(f"route_pattern                  : {self.route_pattern}")
        self.get_logger().info(f"require_origin_before_takeoff  : {self.require_origin_before_takeoff}")
        self.get_logger().info(f"origin_sample_count            : {self.origin_sample_count}")
        self.get_logger().info(f"local_position_timeout_sec     : {self.local_position_timeout_sec}")
        self.get_logger().info(f"setpoint_hz                    : {self.setpoint_hz}")
        self.get_logger().info(f"route_mode                     : {self.route_mode}")
        self.get_logger().info(f"square_side_m                  : {self.square_side_m}")
        self.get_logger().info(f"square_yaws_deg                : {self.square_yaws_deg}")
        self.get_logger().info(f"square_origin_mode             : {self.square_origin_mode}")
        self.get_logger().info(f"yaw_angles_deg                 : {self.yaw_angles_deg}")
        self.get_logger().info(f"yaw_acceptance_deg             : {self.yaw_acceptance_deg}")
        self.get_logger().info(f"yaw_timeout_sec                : {self.yaw_timeout_sec}")
        self.get_logger().info(f"enable_low_altitude_inspection : {self.enable_low_altitude_inspection}")
        self.get_logger().info(f"inspect_altitude_m             : {self.inspect_altitude_m}")
        self.get_logger().info(f"disable_inspection_for_person  : {self.disable_inspection_for_person}")
        self.get_logger().info(f"person_safe_altitude_m         : {self.person_safe_altitude_m}")
        self.get_logger().info(f"finish_mode                    : {self.finish_mode}")
        self.get_logger().info(f"abort_mode                     : {self.abort_mode}")

    def route_offsets(self):
        corner_offset_m = self.corner_offset_m
        if corner_offset_m <= 0.0:
            corner_offset_m = self.scout_radius_m / math.sqrt(2.0)

        mapping = {
            "N": (self.scout_radius_m, 0.0),
            "NE": (corner_offset_m, corner_offset_m),
            "E": (0.0, self.scout_radius_m),
            "SE": (-corner_offset_m, corner_offset_m),
            "S": (-self.scout_radius_m, 0.0),
            "SW": (-corner_offset_m, -corner_offset_m),
            "W": (0.0, -self.scout_radius_m),
            "NW": (corner_offset_m, -corner_offset_m),
            "CENTER": (0.0, 0.0),
        }
        route = []
        for token in self.route_pattern.split(","):
            name = token.strip().upper()
            if not name:
                continue
            if name not in mapping:
                self.get_logger().warn(f"Ignoring unknown route token: {name}")
                continue
            north_m, east_m = mapping[name]
            route.append((name, north_m, east_m))
        return route

    def current_radius_m(self) -> float:
        return self.local_ned.current_radius_m()

    def current_altitude_m(self) -> float:
        return self.local_ned.current_altitude_m()

    def local_position_fresh(self, now: float) -> bool:
        return self.local_ned.local_position_fresh(now)

    @staticmethod
    def target_inside_radius(north_m: float, east_m: float, radius_m: float) -> bool:
        return LocalNedController.target_inside_radius(north_m, east_m, radius_m)

    @staticmethod
    def find_candidate(candidates, candidate_index):
        return next(
            (
                candidate
                for candidate in candidates
                if candidate.candidate_index == candidate_index
            ),
            None,
        )

    @staticmethod
    def is_person_candidate(candidate):
        label = candidate.target_label.strip().lower()
        return "person" in label or "human" in label

    @staticmethod
    def parse_float_list(value: str):
        if not value.strip():
            return []
        return [float(item.strip()) for item in value.split(",") if item.strip()]


def main(args=None):
    rclpy.init(args=args)
    node = BoundedScoutMissionNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
