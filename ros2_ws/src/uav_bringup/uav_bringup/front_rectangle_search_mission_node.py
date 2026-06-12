#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from rclpy.node import Node
from uav_interfaces.msg import TargetFeedback

from uav_bringup.common.local_ned_controller import LocalNedController
from uav_bringup.common.mission_config import declare_guided_takeoff_params
from uav_bringup.common.mission_config import load_guided_takeoff_config
from uav_bringup.common.mission_runtime import MissionRuntime


"""
Mission flow:
1. Connect MAVLink, request LOCAL_NED/ATTITUDE telemetry, and wait for readiness.
2. Capture LOCAL_NED origin before takeoff so the rectangle starts at the ground start point.
3. GUIDED takeoff to altitude_m, then capture the front yaw from current attitude by default.
4. Fly a front-facing rectangular serpentine route at fixed altitude and fixed yaw.
5. Optionally execute short, bounded body-frame steps from /uav/vision/target_feedback.
6. Return to the captured LOCAL_NED origin and switch to finish_mode, normally LOITER or LAND.
"""

# LOCAL_NED uses x=North, y=East, z=Down; altitude above home is negative z.


class FrontRectangleSearchMissionNode(Node):
    """
    Search a UAV-front rectangular area using deterministic LOCAL_NED waypoints.

    Target-feedback movement is opt-in and remains constrained by fixed altitude,
    software radius, step clamps, cooldown, and max movement counts.
    """

    def __init__(self):
        super().__init__("front_rectangle_search_mission_node")

        declare_guided_takeoff_params(self, include_land_after_hold=True, default_altitude_m=3.5)
        self.declare_search_params()

        self.config = load_guided_takeoff_config(self, include_land_after_hold=True)
        self.load_search_params()

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
            yaw_log_interval_sec=1.0,
        )
        self.front_yaw_deg_active = None

        self.latest_feedback = None
        self.latest_feedback_time = 0.0
        self.last_feedback_move_time = 0.0
        self.total_feedback_moves = 0

        self.create_subscription(
            TargetFeedback,
            self.target_feedback_topic,
            self.on_target_feedback,
            10,
        )

        self.run_once()

    def declare_search_params(self):
        # Front-rectangle search geometry in body frame: forward is UAV front, right is UAV right.
        self.declare_parameter("front_length_m", 8.0)
        self.declare_parameter("front_width_m", 4.0)
        self.declare_parameter("front_lane_count", 3)
        self.declare_parameter("front_points_per_lane", 3)
        self.declare_parameter("use_current_yaw_as_front", True)
        self.declare_parameter("front_yaw_deg", 0.0)

        # LOCAL_NED reference, route bounds, and waypoint movement control.
        self.declare_parameter("require_origin_before_takeoff", True)
        self.declare_parameter("origin_sample_count", 5)  # LOCAL_NED samples averaged before route generation.
        self.declare_parameter("software_radius_m", 10.0)
        self.declare_parameter("waypoint_acceptance_m", 0.8)
        self.declare_parameter("waypoint_timeout_sec", 30.0)
        self.declare_parameter("setpoint_hz", 2.0)  # Position setpoint send rate to Pixhawk (Hz).
        self.declare_parameter("local_position_timeout_sec", 5.0)

        # Fixed-yaw scan timing and target-feedback observation.
        self.declare_parameter("route_hold_sec", 2.0)
        self.declare_parameter("yaw_speed_deg_s", 20.0)  # (deg/s)
        self.declare_parameter("yaw_acceptance_deg", 8.0)
        self.declare_parameter("yaw_timeout_sec", 8.0)
        self.declare_parameter("target_feedback_topic", "/uav/vision/target_feedback")
        self.declare_parameter("target_feedback_timeout_sec", 5.0)
        self.declare_parameter("stop_on_ready_to_inspect", False)

        # Optional closed-loop target-feedback movement; kept off by default for safety.
        self.declare_parameter("enable_target_feedback_movement", False)
        self.declare_parameter("max_feedback_step_m", 0.5)
        self.declare_parameter("feedback_move_acceptance_m", 0.4)
        self.declare_parameter("feedback_move_timeout_sec", 10.0)
        self.declare_parameter("feedback_move_cooldown_sec", 2.0)
        self.declare_parameter("max_feedback_moves_per_waypoint", 1)
        self.declare_parameter("max_total_feedback_moves", 5)

        # Mission exit behavior.
        self.declare_parameter("finish_mode", "LOITER")
        self.declare_parameter("abort_mode", "RTL")  # Return to Launch

    def load_search_params(self):
        self.front_length_m = float(self.get_parameter("front_length_m").value)
        self.front_width_m = float(self.get_parameter("front_width_m").value)
        self.front_lane_count = max(int(self.get_parameter("front_lane_count").value), 1)
        self.front_points_per_lane = max(int(self.get_parameter("front_points_per_lane").value), 2)
        self.use_current_yaw_as_front = bool(self.get_parameter("use_current_yaw_as_front").value)
        self.front_yaw_deg = float(self.get_parameter("front_yaw_deg").value)

        self.require_origin_before_takeoff = bool(self.get_parameter("require_origin_before_takeoff").value)
        self.origin_sample_count = max(int(self.get_parameter("origin_sample_count").value), 1)
        self.software_radius_m = float(self.get_parameter("software_radius_m").value)
        self.waypoint_acceptance_m = float(self.get_parameter("waypoint_acceptance_m").value)
        self.waypoint_timeout_sec = float(self.get_parameter("waypoint_timeout_sec").value)
        self.setpoint_hz = float(self.get_parameter("setpoint_hz").value)
        self.local_position_timeout_sec = float(self.get_parameter("local_position_timeout_sec").value)

        self.route_hold_sec = float(self.get_parameter("route_hold_sec").value)
        self.yaw_speed_deg_s = float(self.get_parameter("yaw_speed_deg_s").value)
        self.yaw_acceptance_deg = float(self.get_parameter("yaw_acceptance_deg").value)
        self.yaw_timeout_sec = float(self.get_parameter("yaw_timeout_sec").value)
        self.target_feedback_topic = str(self.get_parameter("target_feedback_topic").value)
        self.target_feedback_timeout_sec = float(self.get_parameter("target_feedback_timeout_sec").value)
        self.stop_on_ready_to_inspect = bool(self.get_parameter("stop_on_ready_to_inspect").value)
        self.enable_target_feedback_movement = bool(self.get_parameter("enable_target_feedback_movement").value)
        self.max_feedback_step_m = float(self.get_parameter("max_feedback_step_m").value)
        self.feedback_move_acceptance_m = float(self.get_parameter("feedback_move_acceptance_m").value)
        self.feedback_move_timeout_sec = float(self.get_parameter("feedback_move_timeout_sec").value)
        self.feedback_move_cooldown_sec = float(self.get_parameter("feedback_move_cooldown_sec").value)
        self.max_feedback_moves_per_waypoint = max(
            int(self.get_parameter("max_feedback_moves_per_waypoint").value),
            0,
        )
        self.max_total_feedback_moves = max(
            int(self.get_parameter("max_total_feedback_moves").value),
            0,
        )

        self.finish_mode = str(self.get_parameter("finish_mode").value).upper()
        self.abort_mode = str(self.get_parameter("abort_mode").value).upper()

    def run_once(self):
        self.runtime.log_config("=== Front Rectangle Search Mission ===", include_land_after_hold=True)
        self.log_search_config()

        if not self.runtime.connect_and_prepare(stream_hz=5.0, include_flow_rad=True):
            self.runtime.finish()
            return

        self.request_search_streams()
        self.flight_ops = self.runtime.make_flight_ops(
            drain_fn=self.local_ned.drain_messages,
        )

        if self.runtime.stop_if_dry_run("Dry-run enabled. Front rectangle movement/arm/takeoff commands not sent."):
            return

        origin_captured = self.capture_local_origin("pre_takeoff")
        if not origin_captured and self.require_origin_before_takeoff:
            self.abort_mission("pre_takeoff_origin_failed", set_abort_mode=False)
            return

        if not self.takeoff_to_search_altitude():
            self.abort_mission("takeoff_failed")
            return

        if not origin_captured and not self.capture_local_origin("post_takeoff"):
            self.abort_mission("local_origin_failed")
            return

        if not self.capture_front_yaw():
            self.abort_mission("front_yaw_capture_failed")
            return

        if not self.set_yaw(self.front_yaw_deg_active):
            self.abort_mission("front_yaw_set_failed")
            return

        if not self.run_front_rectangle_route():
            self.abort_mission("front_rectangle_route_failed")
            return

        if not self.goto_offset("CENTER_RETURN", 0.0, 0.0, self.config.altitude_m):
            self.abort_mission("return_center_failed")
            return

        self.finish_mission()

    def takeoff_to_search_altitude(self) -> bool:
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

    def run_front_rectangle_route(self) -> bool:
        route = self.front_rectangle_offsets()
        if not route:
            self.get_logger().error("Front rectangle route is empty.")
            return False

        for label, north_m, east_m in route:
            if not self.set_yaw(self.front_yaw_deg_active):
                return False
            if not self.goto_offset(label, north_m, east_m, self.config.altitude_m):
                return False
            if not self.hold_and_watch_feedback(label=label, hold_sec=self.route_hold_sec):
                return False
            if self.stop_on_ready_to_inspect and self.feedback_ready_to_inspect():
                self.get_logger().warn("Stopping search: target feedback is ready_to_inspect.")
                return True

        return True

    def front_rectangle_offsets(self):
        lanes = self.lane_offsets(self.front_width_m, self.front_lane_count)
        forward_values = [
            self.front_length_m * index / (self.front_points_per_lane - 1)
            for index in range(self.front_points_per_lane)
        ]

        route = []
        waypoint_index = 1
        for lane_index, right_m in enumerate(lanes):
            lane_forward_values = forward_values
            if lane_index % 2 == 1:
                lane_forward_values = list(reversed(forward_values))

            for forward_m in lane_forward_values:
                north_m, east_m = self.body_to_local_offset(forward_m, right_m)
                label = f"FRONT_RECT_{waypoint_index:02d}_F{forward_m:.1f}_R{right_m:.1f}"
                if not self.target_inside_radius(north_m, east_m, self.software_radius_m):
                    self.get_logger().error(
                        f"Generated waypoint outside software radius: {label} "
                        f"radius={math.hypot(north_m, east_m):.2f}m"
                    )
                    return []
                route.append((label, north_m, east_m))
                waypoint_index += 1

        return route

    def body_to_local_offset(self, forward_m: float, right_m: float):
        return self.local_ned.body_to_local_offset(
            forward_m,
            right_m,
            self.front_yaw_deg_active,
        )

    @staticmethod
    def lane_offsets(width_m: float, lane_count: int):
        if lane_count <= 1:
            return [0.0]
        return [
            -width_m / 2.0 + width_m * index / (lane_count - 1)
            for index in range(lane_count)
        ]

    def goto_offset(
        self,
        label: str,
        north_m: float,
        east_m: float,
        altitude_m: float,
        acceptance_m: float = None,
        timeout_sec: float = None,
    ) -> bool:
        return self.local_ned.goto_offset(
            label,
            north_m,
            east_m,
            altitude_m,
            acceptance_m=acceptance_m,
            timeout_sec=timeout_sec,
        )

    def hold_and_watch_feedback(self, *, label: str, hold_sec: float) -> bool:
        start = time.monotonic()
        last_print = 0.0
        moves_this_waypoint = 0

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
                self.log_hold_status(label, now)
                last_print = now

            if self.stop_on_ready_to_inspect and self.feedback_ready_to_inspect():
                return True

            if self.should_execute_feedback_movement(now, moves_this_waypoint):
                if not self.execute_feedback_movement(label):
                    return False
                moves_this_waypoint += 1
                self.total_feedback_moves += 1
                self.last_feedback_move_time = time.monotonic()
                start = time.monotonic()

            if now - start >= hold_sec:
                return True

            time.sleep(0.05)

        return False

    def log_hold_status(self, label: str, now: float):
        feedback_summary = "feedback=none"
        if self.latest_feedback is not None:
            age_sec = now - self.latest_feedback_time
            feedback_summary = (
                f"feedback_age={age_sec:.1f}s motion={self.latest_feedback.motion_command} "
                f"track={self.latest_feedback.track_id} ready={self.latest_feedback.ready_to_inspect}"
            )

        self.get_logger().info(
            f"Hold {label}: radius={self.current_radius_m():.2f}m "
            f"alt={self.current_altitude_m():.2f}m {feedback_summary}"
        )

    def feedback_ready_to_inspect(self) -> bool:
        if self.latest_feedback is None:
            return False
        if time.monotonic() - self.latest_feedback_time > self.target_feedback_timeout_sec:
            return False
        return bool(self.latest_feedback.has_target and self.latest_feedback.ready_to_inspect)

    def should_execute_feedback_movement(self, now: float, moves_this_waypoint: int) -> bool:
        if not self.enable_target_feedback_movement:
            return False
        if self.latest_feedback is None:
            return False
        if now - self.latest_feedback_time > self.target_feedback_timeout_sec:
            return False
        if moves_this_waypoint >= self.max_feedback_moves_per_waypoint:
            return False
        if self.max_total_feedback_moves > 0 and self.total_feedback_moves >= self.max_total_feedback_moves:
            return False
        if now - self.last_feedback_move_time < self.feedback_move_cooldown_sec:
            return False
        if not self.latest_feedback.has_target or self.latest_feedback.ready_to_inspect:
            return False

        motion_command = self.latest_feedback.motion_command.upper()
        if motion_command not in {"APPROACH", "STRAFE_LEFT", "STRAFE_RIGHT"}:
            return False

        forward_m, right_m = self.clamped_feedback_step(self.latest_feedback)
        return math.hypot(forward_m, right_m) > 0.01

    def execute_feedback_movement(self, label: str) -> bool:
        feedback = self.latest_feedback
        forward_m, right_m = self.clamped_feedback_step(feedback)
        delta_north_m, delta_east_m = self.body_to_local_offset(forward_m, right_m)

        current_north_m, current_east_m = self.local_ned.current_offset()
        target_north_m = current_north_m + delta_north_m
        target_east_m = current_east_m + delta_east_m

        if not self.target_inside_radius(target_north_m, target_east_m, self.software_radius_m):
            self.get_logger().warn(
                "Skipping target-feedback movement outside software radius: "
                f"target_radius={math.hypot(target_north_m, target_east_m):.2f}m "
                f"> software_radius_m={self.software_radius_m:.2f}m"
            )
            return True

        move_label = (
            f"{label}/FEEDBACK_{feedback.motion_command}_"
            f"T{feedback.track_id}_F{forward_m:.2f}_R{right_m:.2f}"
        )
        self.get_logger().warn(
            f"Target-feedback move: {move_label}, "
            f"body_forward={forward_m:.2f}m body_right={right_m:.2f}m"
        )
        return self.goto_offset(
            move_label,
            target_north_m,
            target_east_m,
            self.config.altitude_m,
            acceptance_m=self.feedback_move_acceptance_m,
            timeout_sec=self.feedback_move_timeout_sec,
        )

    def clamped_feedback_step(self, feedback):
        return (
            self.clamp(
                float(feedback.recommended_body_forward_m),
                -self.max_feedback_step_m,
                self.max_feedback_step_m,
            ),
            self.clamp(
                float(feedback.recommended_body_right_m),
                -self.max_feedback_step_m,
                self.max_feedback_step_m,
            ),
        )

    def capture_front_yaw(self) -> bool:
        if not self.use_current_yaw_as_front:
            self.front_yaw_deg_active = self.local_ned.normalize_angle_deg(self.front_yaw_deg)
            self.get_logger().warn(f"Front yaw fixed from parameter: {self.front_yaw_deg_active:.1f} deg")
            return True

        start = time.monotonic()
        while time.monotonic() - start < self.local_position_timeout_sec:
            self.spin_and_drain()
            now = time.monotonic()
            if self.local_ned.yaw_fresh(now):
                self.front_yaw_deg_active = self.local_ned.current_yaw_deg
                self.get_logger().warn(f"Front yaw captured from current attitude: {self.front_yaw_deg_active:.1f} deg")
                return True
            time.sleep(0.05)

        self.get_logger().error("Failed to capture current yaw as front direction.")
        return False

    def set_yaw(self, yaw_deg: float) -> bool:
        """Ask ArduPilot to rotate toward the target yaw while staying in GUIDED mode."""

        return self.local_ned.set_yaw(yaw_deg)

    def request_search_streams(self):
        self.local_ned.request_streams(local_position_hz=5.0, attitude_hz=10.0)

    def capture_local_origin(self, phase: str) -> bool:
        return self.local_ned.capture_origin(phase)

    def on_target_feedback(self, msg):
        self.latest_feedback = msg
        self.latest_feedback_time = time.monotonic()

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

    def log_search_config(self):
        self.get_logger().info(f"front_length_m               : {self.front_length_m}")
        self.get_logger().info(f"front_width_m                : {self.front_width_m}")
        self.get_logger().info(f"front_lane_count             : {self.front_lane_count}")
        self.get_logger().info(f"front_points_per_lane        : {self.front_points_per_lane}")
        self.get_logger().info(f"use_current_yaw_as_front     : {self.use_current_yaw_as_front}")
        self.get_logger().info(f"front_yaw_deg                : {self.front_yaw_deg}")
        self.get_logger().info(f"software_radius_m            : {self.software_radius_m}")
        self.get_logger().info(f"waypoint_acceptance_m        : {self.waypoint_acceptance_m}")
        self.get_logger().info(f"setpoint_hz                  : {self.setpoint_hz}")
        self.get_logger().info(f"route_hold_sec               : {self.route_hold_sec}")
        self.get_logger().info(f"target_feedback_topic        : {self.target_feedback_topic}")
        self.get_logger().info(f"stop_on_ready_to_inspect     : {self.stop_on_ready_to_inspect}")
        self.get_logger().info(f"enable_target_feedback_move  : {self.enable_target_feedback_movement}")
        self.get_logger().info(f"max_feedback_step_m          : {self.max_feedback_step_m}")
        self.get_logger().info(f"max_total_feedback_moves     : {self.max_total_feedback_moves}")
        self.get_logger().info(f"finish_mode                  : {self.finish_mode}")
        self.get_logger().info(f"abort_mode                   : {self.abort_mode}")

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
    def clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min(value, max_value), min_value)


def main(args=None):
    rclpy.init(args=args)
    node = FrontRectangleSearchMissionNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
