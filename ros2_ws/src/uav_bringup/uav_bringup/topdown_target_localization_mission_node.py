#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from std_msgs.msg import String
from std_msgs.msg import UInt16
from uav_interfaces.msg import GeminiReport
from uav_interfaces.msg import TargetFeedback
from uav_interfaces.msg import TargetPositionEstimate
from uav_interfaces.msg import TopdownCenteringFeedback

from uav_bringup.common.local_ned_controller import LocalNedController
from uav_bringup.common.mission_config import declare_guided_takeoff_params
from uav_bringup.common.mission_config import load_guided_takeoff_config
from uav_bringup.common.mission_runtime import MissionRuntime


class TopdownTargetLocalizationMissionNode(Node):
    """
    Search, center above a VLM-selected target, fix its RTK position, and return.

    Flight flow:
    PREPARE -> TAKEOFF -> SEARCH_ROUTE -> LOITER -> GIMBAL_TOPDOWN
    -> GUIDED_CORRECTIONS/LOITER -> RTK_LOCALIZE -> RETURN_START -> LOITER.
    Any safety-critical failure transitions to the configured abort mode.
    """

    def __init__(self):
        super().__init__("topdown_target_localization_mission")

        declare_guided_takeoff_params(self, default_altitude_m=3.5)
        self.declare_mission_params()

        self.config = load_guided_takeoff_config(self)
        self.load_mission_params()

        self.stop_event = threading.Event()
        self.runtime = MissionRuntime(
            node=self,
            config=self.config,
            stop_event=self.stop_event,
        )
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
        )

        self.front_yaw_deg_active = None
        self.latest_front_feedback = None
        self.latest_front_feedback_time = 0.0
        self.latest_front_detection = False
        self.latest_front_detection_time = 0.0
        self.latest_topdown_feedback = None
        self.latest_topdown_feedback_time = 0.0
        self.latest_target_estimate = None
        self.latest_target_estimate_time = 0.0
        self.topdown_active = False
        self.last_gimbal_publish_time = 0.0
        self.total_front_moves = 0
        self.total_topdown_moves = 0
        self.current_state = "INITIALIZING"

        self.state_pub = self.create_publisher(String, self.mission_state_topic, 10)
        self.create_subscription(
            GeminiReport,
            self.gemini_report_topic,
            self.on_gemini_report,
            10,
        )
        self.create_subscription(
            TargetFeedback,
            self.front_feedback_topic,
            self.on_front_feedback,
            10,
        )
        self.create_subscription(
            TopdownCenteringFeedback,
            self.topdown_feedback_topic,
            self.on_topdown_feedback,
            10,
        )
        self.create_subscription(
            TargetPositionEstimate,
            self.target_estimate_topic,
            self.on_target_estimate,
            10,
        )
        self.gimbal_pub = self.create_publisher(
            UInt16,
            self.gimbal_pitch_target_topic,
            10,
        )
        self.refresh_pub = self.create_publisher(
            Empty,
            self.frame_selection_trigger_topic,
            10,
        )

        self.run_once()

    def declare_mission_params(self):
        # Search rectangle in the captured UAV-front body frame.
        self.declare_parameter("front_length_m", 8.0)
        self.declare_parameter("front_width_m", 4.0)
        self.declare_parameter("front_lane_count", 3)
        self.declare_parameter("front_points_per_lane", 3)
        self.declare_parameter("route_hold_sec", 2.0)
        self.declare_parameter("use_current_yaw_as_front", True)
        self.declare_parameter("front_yaw_deg", 0.0)

        # LOCAL_NED reference and shared movement safety.
        self.declare_parameter("origin_sample_count", 5)
        self.declare_parameter("software_radius_m", 10.0)
        self.declare_parameter("waypoint_acceptance_m", 0.8)
        self.declare_parameter("waypoint_timeout_sec", 30.0)
        self.declare_parameter("setpoint_hz", 2.0)
        self.declare_parameter("local_position_timeout_sec", 5.0)
        self.declare_parameter("yaw_speed_deg_s", 20.0)
        self.declare_parameter("yaw_acceptance_deg", 8.0)
        self.declare_parameter("yaw_timeout_sec", 8.0)

        # Front-view target approach before switching the gimbal to top-down.
        self.declare_parameter("front_feedback_topic", "/uav/vision/target_feedback")
        self.declare_parameter("gemini_report_topic", "/uav/vision/gemini_report")
        self.declare_parameter("front_feedback_timeout_sec", 5.0)
        self.declare_parameter("search_feedback_wait_sec", 10.0)
        self.declare_parameter("front_max_step_m", 0.5)
        self.declare_parameter("front_move_acceptance_m", 0.4)
        self.declare_parameter("front_move_timeout_sec", 10.0)
        self.declare_parameter("front_max_total_moves", 8)

        # Gimbal command is parameterized because the calibrated top-down PWM is unknown.
        self.declare_parameter("gimbal_pitch_target_topic", "/uav/gimbal/pitch_target_pwm")
        self.declare_parameter("gimbal_pwm_min", 1550)
        self.declare_parameter("gimbal_pwm_max", 1580)
        self.declare_parameter("topdown_gimbal_pwm", 0)
        self.declare_parameter("return_gimbal_pwm", 1550)
        self.declare_parameter("gimbal_command_period_sec", 1.0)
        self.declare_parameter("topdown_gimbal_settle_sec", 2.0)
        self.declare_parameter(
            "frame_selection_trigger_topic",
            "/uav/vision/select_frame_trigger",
        )
        self.declare_parameter("request_fresh_detection_during_search", True)
        self.declare_parameter("request_fresh_detection_after_topdown", True)

        # Top-down closed-loop correction and localization limits.
        self.declare_parameter(
            "topdown_feedback_topic",
            "/uav/vision/topdown_centering_feedback",
        )
        self.declare_parameter(
            "target_estimate_topic",
            "/uav/vision/target_position_estimate",
        )
        self.declare_parameter("topdown_feedback_timeout_sec", 2.0)
        self.declare_parameter("topdown_initial_feedback_timeout_sec", 30.0)
        self.declare_parameter("topdown_post_move_feedback_timeout_sec", 5.0)
        self.declare_parameter("topdown_total_timeout_sec", 90.0)
        self.declare_parameter("topdown_max_step_m", 0.30)
        self.declare_parameter("topdown_move_acceptance_m", 0.25)
        self.declare_parameter("topdown_move_timeout_sec", 10.0)
        self.declare_parameter("topdown_move_cooldown_sec", 1.0)
        self.declare_parameter("topdown_max_moves", 20)
        self.declare_parameter("target_estimate_timeout_sec", 8.0)

        self.declare_parameter("completion_mode", "LOITER")
        self.declare_parameter("abort_mode", "RTL")
        self.declare_parameter("mission_state_topic", "/uav/mission/state")

    def load_mission_params(self):
        self.front_length_m = float(self.get_parameter("front_length_m").value)
        self.front_width_m = float(self.get_parameter("front_width_m").value)
        self.front_lane_count = max(int(self.get_parameter("front_lane_count").value), 1)
        self.front_points_per_lane = max(
            int(self.get_parameter("front_points_per_lane").value),
            2,
        )
        self.route_hold_sec = max(float(self.get_parameter("route_hold_sec").value), 0.0)
        self.use_current_yaw_as_front = bool(
            self.get_parameter("use_current_yaw_as_front").value
        )
        self.front_yaw_deg = float(self.get_parameter("front_yaw_deg").value)

        self.origin_sample_count = max(int(self.get_parameter("origin_sample_count").value), 1)
        self.software_radius_m = float(self.get_parameter("software_radius_m").value)
        self.waypoint_acceptance_m = float(self.get_parameter("waypoint_acceptance_m").value)
        self.waypoint_timeout_sec = float(self.get_parameter("waypoint_timeout_sec").value)
        self.setpoint_hz = float(self.get_parameter("setpoint_hz").value)
        self.local_position_timeout_sec = float(
            self.get_parameter("local_position_timeout_sec").value
        )
        self.yaw_speed_deg_s = float(self.get_parameter("yaw_speed_deg_s").value)
        self.yaw_acceptance_deg = float(self.get_parameter("yaw_acceptance_deg").value)
        self.yaw_timeout_sec = float(self.get_parameter("yaw_timeout_sec").value)

        self.front_feedback_topic = str(self.get_parameter("front_feedback_topic").value)
        self.gemini_report_topic = str(self.get_parameter("gemini_report_topic").value)
        self.front_feedback_timeout_sec = float(
            self.get_parameter("front_feedback_timeout_sec").value
        )
        self.search_feedback_wait_sec = max(
            float(self.get_parameter("search_feedback_wait_sec").value),
            0.0,
        )
        self.front_max_step_m = float(self.get_parameter("front_max_step_m").value)
        self.front_move_acceptance_m = float(
            self.get_parameter("front_move_acceptance_m").value
        )
        self.front_move_timeout_sec = float(
            self.get_parameter("front_move_timeout_sec").value
        )
        self.front_max_total_moves = max(
            int(self.get_parameter("front_max_total_moves").value),
            0,
        )

        self.gimbal_pitch_target_topic = str(
            self.get_parameter("gimbal_pitch_target_topic").value
        )
        self.gimbal_pwm_min = int(self.get_parameter("gimbal_pwm_min").value)
        self.gimbal_pwm_max = int(self.get_parameter("gimbal_pwm_max").value)
        self.topdown_gimbal_pwm = int(self.get_parameter("topdown_gimbal_pwm").value)
        self.return_gimbal_pwm = int(self.get_parameter("return_gimbal_pwm").value)
        self.gimbal_command_period_sec = max(
            float(self.get_parameter("gimbal_command_period_sec").value),
            0.1,
        )
        self.topdown_gimbal_settle_sec = max(
            float(self.get_parameter("topdown_gimbal_settle_sec").value),
            0.0,
        )
        self.frame_selection_trigger_topic = str(
            self.get_parameter("frame_selection_trigger_topic").value
        )
        self.request_fresh_detection_during_search = bool(
            self.get_parameter("request_fresh_detection_during_search").value
        )
        self.request_fresh_detection_after_topdown = bool(
            self.get_parameter("request_fresh_detection_after_topdown").value
        )

        self.topdown_feedback_topic = str(
            self.get_parameter("topdown_feedback_topic").value
        )
        self.target_estimate_topic = str(
            self.get_parameter("target_estimate_topic").value
        )
        self.topdown_feedback_timeout_sec = float(
            self.get_parameter("topdown_feedback_timeout_sec").value
        )
        self.topdown_initial_feedback_timeout_sec = float(
            self.get_parameter("topdown_initial_feedback_timeout_sec").value
        )
        self.topdown_post_move_feedback_timeout_sec = float(
            self.get_parameter("topdown_post_move_feedback_timeout_sec").value
        )
        self.topdown_total_timeout_sec = float(
            self.get_parameter("topdown_total_timeout_sec").value
        )
        self.topdown_max_step_m = float(self.get_parameter("topdown_max_step_m").value)
        self.topdown_move_acceptance_m = float(
            self.get_parameter("topdown_move_acceptance_m").value
        )
        self.topdown_move_timeout_sec = float(
            self.get_parameter("topdown_move_timeout_sec").value
        )
        self.topdown_move_cooldown_sec = max(
            float(self.get_parameter("topdown_move_cooldown_sec").value),
            0.0,
        )
        self.topdown_max_moves = max(int(self.get_parameter("topdown_max_moves").value), 0)
        self.target_estimate_timeout_sec = float(
            self.get_parameter("target_estimate_timeout_sec").value
        )
        self.completion_mode = str(self.get_parameter("completion_mode").value).upper()
        self.abort_mode = str(self.get_parameter("abort_mode").value).upper()
        self.mission_state_topic = str(self.get_parameter("mission_state_topic").value)

    def run_once(self):
        self.set_state("PREPARE")
        self.runtime.log_config("=== Top-down Target Localization Mission ===")
        self.log_mission_config()

        if not self.gimbal_pwm_config_valid():
            self.runtime.finish()
            return

        if not self.runtime.connect_and_prepare(stream_hz=5.0, include_flow_rad=True):
            self.runtime.finish()
            return

        self.local_ned.request_streams(local_position_hz=10.0, attitude_hz=10.0)
        self.flight_ops = self.runtime.make_flight_ops()

        if self.runtime.stop_if_dry_run(
            "Dry-run enabled. Mission flight and gimbal commands not sent."
        ):
            return

        if not self.local_ned.capture_origin("pre_takeoff"):
            self.abort_mission("pre_takeoff_origin_failed", set_abort_mode=False)
            return
        self.set_state("TAKEOFF")
        if not self.takeoff():
            self.abort_mission("takeoff_failed")
            return
        if not self.capture_front_yaw():
            self.abort_mission("front_yaw_capture_failed")
            return
        if not self.local_ned.set_yaw(self.front_yaw_deg_active):
            self.abort_mission("front_yaw_set_failed")
            return

        self.set_state("SEARCH_ROUTE")
        if not self.search_until_target_ready():
            self.abort_mission("target_not_ready_during_search")
            return
        if not self.run_topdown_localization():
            self.abort_mission("topdown_localization_failed")
            return
        if not self.return_to_start():
            self.abort_mission("return_start_failed")
            return

        self.finish_mission()

    def takeoff(self):
        if not self.flight_ops.set_mode("GUIDED"):
            return False
        if not self.flight_ops.arm() or not self.flight_ops.wait_armed():
            return False
        if not self.flight_ops.takeoff(self.config.altitude_m):
            return False
        return self.flight_ops.wait_altitude_reached(
            self.config.altitude_m,
            self.config.altitude_ratio,
        )

    def search_until_target_ready(self):
        for label, north_m, east_m in self.front_rectangle_offsets():
            if not self.local_ned.set_yaw(self.front_yaw_deg_active):
                return False
            if not self.local_ned.goto_offset(label, north_m, east_m, self.config.altitude_m):
                return False
            hold_result = self.hold_search_waypoint(label)
            if hold_result == "TARGET_READY":
                return True
            if hold_result == "FAIL":
                return False
        return False

    def hold_search_waypoint(self, label):
        if self.request_fresh_detection_during_search:
            self.latest_front_feedback = None
            self.latest_front_detection = False
            self.request_fresh_frame_selection("search waypoint")

        started_at = time.monotonic()
        hold_limit_sec = max(self.route_hold_sec, self.search_feedback_wait_sec)
        while rclpy.ok() and not self.stop_event.is_set():
            now = time.monotonic()
            self.spin_and_drain()
            if not self.safety_state_valid(now):
                return "FAIL"
            if self.front_feedback_ready(now):
                self.get_logger().warn(f"Front target ready at {label}.")
                return "TARGET_READY"
            if self.front_feedback_move_available(now):
                if not self.execute_front_feedback_move(label):
                    return "FAIL"
                started_at = time.monotonic()
            if now - started_at >= hold_limit_sec:
                return "CONTINUE"
            time.sleep(0.05)
        return "FAIL"

    def run_topdown_localization(self):
        self.set_state("LOITER_BEFORE_TOPDOWN")
        if not self.flight_ops.set_mode("LOITER"):
            return False

        self.set_state("GIMBAL_TOPDOWN")
        self.topdown_active = True
        self.publish_gimbal_pwm(self.topdown_gimbal_pwm, force=True)
        if not self.wait_with_topdown_gimbal(self.topdown_gimbal_settle_sec):
            return False

        self.latest_topdown_feedback = None
        self.latest_target_estimate = None
        if self.request_fresh_detection_after_topdown:
            self.refresh_pub.publish(Empty())
            self.get_logger().warn("Requested fresh frame selection after top-down settle.")

        if not self.wait_for_initial_topdown_feedback():
            return False

        started_at = time.monotonic()
        last_move_at = 0.0
        while rclpy.ok() and not self.stop_event.is_set():
            now = time.monotonic()
            self.spin_and_drain()
            self.publish_gimbal_pwm(self.topdown_gimbal_pwm)

            if not self.safety_state_valid(now):
                return False
            if now - started_at > self.topdown_total_timeout_sec:
                self.get_logger().error("Top-down localization total timeout.")
                return False
            if not self.topdown_feedback_fresh(now):
                self.get_logger().error("Top-down feedback became stale.")
                return False

            feedback = self.latest_topdown_feedback
            if feedback.ready_to_localize:
                self.set_state("RTK_LOCALIZE")
                if not self.flight_ops.set_mode("LOITER"):
                    return False
                return self.wait_for_target_estimate()

            if not feedback.has_target:
                self.get_logger().error(f"Top-down tracking unavailable: {feedback.status}")
                return False
            if self.topdown_max_moves > 0 and self.total_topdown_moves >= self.topdown_max_moves:
                self.get_logger().error("Top-down correction move limit reached.")
                return False
            if now - last_move_at < self.topdown_move_cooldown_sec:
                time.sleep(0.05)
                continue

            if not self.execute_topdown_correction(feedback):
                return False
            self.total_topdown_moves += 1
            last_move_at = time.monotonic()

        return False

    def execute_topdown_correction(self, feedback):
        forward_m = self.clamp(
            float(feedback.recommended_body_forward_m),
            -self.topdown_max_step_m,
            self.topdown_max_step_m,
        )
        right_m = self.clamp(
            float(feedback.recommended_body_right_m),
            -self.topdown_max_step_m,
            self.topdown_max_step_m,
        )
        if math.hypot(forward_m, right_m) < 0.01:
            time.sleep(0.05)
            return True

        target = self.relative_body_target(forward_m, right_m)
        if target is None:
            return False
        target_north_m, target_east_m = target
        if not self.target_inside_radius(target_north_m, target_east_m):
            self.get_logger().error("Top-down correction target exceeds software radius.")
            return False

        if not self.flight_ops.set_mode("GUIDED"):
            return False
        self.set_state("GUIDED_CORRECTION")
        self.publish_gimbal_pwm(self.topdown_gimbal_pwm, force=True)
        moved = self.local_ned.goto_offset(
            f"TOPDOWN_CORRECTION_{self.total_topdown_moves + 1:02d}",
            target_north_m,
            target_east_m,
            self.config.altitude_m,
            acceptance_m=self.topdown_move_acceptance_m,
            timeout_sec=self.topdown_move_timeout_sec,
        )
        if not moved:
            return False
        if not self.flight_ops.set_mode("LOITER"):
            return False
        self.set_state("LOITER_REASSESS")
        self.publish_gimbal_pwm(self.topdown_gimbal_pwm, force=True)
        self.latest_topdown_feedback = None
        return self.wait_for_topdown_feedback(self.topdown_post_move_feedback_timeout_sec)

    def execute_front_feedback_move(self, label):
        feedback = self.latest_front_feedback
        forward_m = self.clamp(
            float(feedback.recommended_body_forward_m),
            -self.front_max_step_m,
            self.front_max_step_m,
        )
        right_m = self.clamp(
            float(feedback.recommended_body_right_m),
            -self.front_max_step_m,
            self.front_max_step_m,
        )
        target = self.relative_body_target(forward_m, right_m)
        if target is None:
            return False
        target_north_m, target_east_m = target
        if not self.target_inside_radius(target_north_m, target_east_m):
            self.get_logger().warn("Skipping front feedback move outside software radius.")
            return True

        self.total_front_moves += 1
        moved = self.local_ned.goto_offset(
            f"{label}/FRONT_FEEDBACK_{self.total_front_moves:02d}",
            target_north_m,
            target_east_m,
            self.config.altitude_m,
            acceptance_m=self.front_move_acceptance_m,
            timeout_sec=self.front_move_timeout_sec,
        )
        if moved and self.request_fresh_detection_during_search:
            self.latest_front_feedback = None
            self.latest_front_detection = False
            self.request_fresh_frame_selection("front feedback move")
        return moved

    def wait_for_initial_topdown_feedback(self):
        return self.wait_for_topdown_feedback(self.topdown_initial_feedback_timeout_sec)

    def wait_for_topdown_feedback(self, timeout_sec):
        started_at = time.monotonic()
        while time.monotonic() - started_at <= timeout_sec:
            self.spin_and_drain()
            self.publish_gimbal_pwm(self.topdown_gimbal_pwm)
            if (
                self.latest_topdown_feedback is not None
                and self.latest_topdown_feedback.has_target
            ):
                return True
            time.sleep(0.05)
        self.get_logger().error("Top-down feedback timeout.")
        return False

    def wait_for_target_estimate(self):
        started_at = time.monotonic()
        while time.monotonic() - started_at <= self.target_estimate_timeout_sec:
            self.spin_and_drain()
            self.publish_gimbal_pwm(self.topdown_gimbal_pwm)
            if self.latest_target_estimate is not None:
                if self.latest_target_estimate.has_estimate:
                    self.get_logger().warn(
                        "Accepted target position estimate: "
                        f"lat={self.latest_target_estimate.estimated_latitude_deg:.9f}, "
                        f"lon={self.latest_target_estimate.estimated_longitude_deg:.9f}, "
                        f"error~{self.latest_target_estimate.estimated_horizontal_error_m:.2f}m"
                    )
                    return True
                self.get_logger().error(
                    f"Target estimate rejected: {self.latest_target_estimate.reason}"
                )
                return False
            time.sleep(0.05)
        self.get_logger().error("Target position estimate timeout.")
        return False

    def return_to_start(self):
        self.set_state("RETURN_START")
        self.topdown_active = False
        self.publish_gimbal_pwm(self.return_gimbal_pwm, force=True)
        if not self.flight_ops.set_mode("GUIDED"):
            return False
        if not self.local_ned.goto_offset(
            "RETURN_START",
            0.0,
            0.0,
            self.config.altitude_m,
        ):
            return False
        return self.flight_ops.set_mode(self.completion_mode)

    def finish_mission(self):
        self.set_state("COMPLETE")
        self.get_logger().warn("Top-down target localization mission completed.")
        self.runtime.finish()

    def abort_mission(self, reason, set_abort_mode=True):
        self.set_state("ABORT")
        self.get_logger().error(f"Mission abort: {reason}")
        self.topdown_active = False
        self.publish_gimbal_pwm(self.return_gimbal_pwm, force=True)
        if set_abort_mode and self.flight_ops is not None and self.abort_mode:
            self.flight_ops.set_mode(self.abort_mode)
        self.runtime.finish()

    def front_rectangle_offsets(self):
        lanes = self.lane_offsets(self.front_width_m, self.front_lane_count)
        forward_values = [
            self.front_length_m * index / (self.front_points_per_lane - 1)
            for index in range(self.front_points_per_lane)
        ]
        route = []
        index = 1
        for lane_index, right_m in enumerate(lanes):
            values = forward_values if lane_index % 2 == 0 else reversed(forward_values)
            for forward_m in values:
                north_m, east_m = self.local_ned.body_to_local_offset(
                    forward_m,
                    right_m,
                    self.front_yaw_deg_active,
                )
                if not self.target_inside_radius(north_m, east_m):
                    self.get_logger().error("Generated search waypoint exceeds software radius.")
                    return []
                route.append((f"SEARCH_{index:02d}", north_m, east_m))
                index += 1
        return route

    def capture_front_yaw(self):
        if not self.use_current_yaw_as_front:
            self.front_yaw_deg_active = self.local_ned.normalize_angle_deg(
                self.front_yaw_deg
            )
            return True

        started_at = time.monotonic()
        while time.monotonic() - started_at <= self.local_position_timeout_sec:
            self.spin_and_drain()
            if self.local_ned.yaw_fresh(time.monotonic()):
                self.front_yaw_deg_active = self.local_ned.current_yaw_deg
                return True
            time.sleep(0.05)
        return False

    def front_feedback_ready(self, now):
        return (
            self.front_detection_fresh(now)
            and self.latest_front_detection
            and self.front_feedback_fresh(now)
            and self.latest_front_feedback.has_target
            and self.latest_front_feedback.ready_to_inspect
        )

    def front_feedback_move_available(self, now):
        if not self.front_detection_fresh(now) or not self.latest_front_detection:
            return False
        if not self.front_feedback_fresh(now):
            return False
        if self.front_max_total_moves > 0 and self.total_front_moves >= self.front_max_total_moves:
            return False
        feedback = self.latest_front_feedback
        return bool(
            feedback.has_target
            and not feedback.ready_to_inspect
            and math.hypot(
                feedback.recommended_body_forward_m,
                feedback.recommended_body_right_m,
            ) > 0.01
        )

    def front_feedback_fresh(self, now):
        return (
            self.latest_front_feedback is not None
            and now - self.latest_front_feedback_time <= self.front_feedback_timeout_sec
        )

    def front_detection_fresh(self, now):
        return now - self.latest_front_detection_time <= self.front_feedback_timeout_sec

    def topdown_feedback_fresh(self, now):
        return (
            self.latest_topdown_feedback is not None
            and now - self.latest_topdown_feedback_time <= self.topdown_feedback_timeout_sec
        )

    def relative_body_target(self, forward_m, right_m):
        current = self.local_ned.current_offset()
        if current is None:
            self.get_logger().error("LOCAL_NED current offset unavailable.")
            return None
        yaw_deg = self.front_yaw_deg_active
        if self.local_ned.yaw_fresh(time.monotonic()):
            yaw_deg = self.local_ned.current_yaw_deg
        delta_north_m, delta_east_m = self.local_ned.body_to_local_offset(
            forward_m,
            right_m,
            yaw_deg,
        )
        return current[0] + delta_north_m, current[1] + delta_east_m

    def safety_state_valid(self, now):
        if not self.local_ned.local_position_fresh(now):
            self.get_logger().error("LOCAL_NED position became stale.")
            return False
        if self.local_ned.current_radius_m() > self.software_radius_m:
            self.get_logger().error("Software radius breach detected.")
            return False
        return True

    def wait_with_topdown_gimbal(self, duration_sec):
        started_at = time.monotonic()
        while time.monotonic() - started_at < duration_sec:
            self.spin_and_drain()
            self.publish_gimbal_pwm(self.topdown_gimbal_pwm)
            if not self.safety_state_valid(time.monotonic()):
                return False
            time.sleep(0.05)
        return True

    def publish_gimbal_pwm(self, pwm, force=False):
        now = time.monotonic()
        if not force and now - self.last_gimbal_publish_time < self.gimbal_command_period_sec:
            return
        msg = UInt16()
        msg.data = int(self.clamp(pwm, 0, 65535))
        self.gimbal_pub.publish(msg)
        self.last_gimbal_publish_time = now

    def request_fresh_frame_selection(self, reason):
        self.refresh_pub.publish(Empty())
        self.get_logger().info(f"Requested fresh frame selection: {reason}")

    def spin_and_drain(self):
        self.local_ned.spin_and_drain()
        if self.topdown_active:
            self.publish_gimbal_pwm(self.topdown_gimbal_pwm)

    def on_front_feedback(self, msg):
        self.latest_front_feedback = msg
        self.latest_front_feedback_time = time.monotonic()

    def on_gemini_report(self, msg):
        self.latest_front_detection = bool(msg.parsed_ok and msg.target_detected)
        self.latest_front_detection_time = time.monotonic()

    def on_topdown_feedback(self, msg):
        self.latest_topdown_feedback = msg
        self.latest_topdown_feedback_time = time.monotonic()

    def on_target_estimate(self, msg):
        self.latest_target_estimate = msg
        self.latest_target_estimate_time = time.monotonic()

    def target_inside_radius(self, north_m, east_m):
        return self.local_ned.target_inside_radius(
            north_m,
            east_m,
            self.software_radius_m,
        )

    def log_mission_config(self):
        self.get_logger().info(f"front_length_m             : {self.front_length_m}")
        self.get_logger().info(f"front_width_m              : {self.front_width_m}")
        self.get_logger().info(f"software_radius_m          : {self.software_radius_m}")
        self.get_logger().info(f"topdown_gimbal_pwm         : {self.topdown_gimbal_pwm}")
        self.get_logger().info(f"gimbal_pwm_range           : {self.gimbal_pwm_min}..{self.gimbal_pwm_max}")
        self.get_logger().info(f"topdown_max_step_m         : {self.topdown_max_step_m}")
        self.get_logger().info(f"topdown_max_moves          : {self.topdown_max_moves}")
        self.get_logger().info(f"topdown_total_timeout_sec  : {self.topdown_total_timeout_sec}")
        self.get_logger().info(f"completion_mode            : {self.completion_mode}")
        self.get_logger().info(f"abort_mode                 : {self.abort_mode}")

    def set_state(self, state):
        self.current_state = state
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        self.get_logger().warn(f"Mission state: {state}")

    def gimbal_pwm_config_valid(self):
        if self.gimbal_pwm_min > self.gimbal_pwm_max:
            self.get_logger().error("Invalid gimbal PWM range: min exceeds max.")
            return False
        for name, value in (
            ("topdown_gimbal_pwm", self.topdown_gimbal_pwm),
            ("return_gimbal_pwm", self.return_gimbal_pwm),
        ):
            if not self.gimbal_pwm_min <= value <= self.gimbal_pwm_max:
                self.get_logger().error(
                    f"{name}={value} is outside configured gimbal PWM range "
                    f"{self.gimbal_pwm_min}..{self.gimbal_pwm_max}."
                )
                return False
        return True

    @staticmethod
    def lane_offsets(width_m, lane_count):
        if lane_count <= 1:
            return [0.0]
        return [
            -width_m / 2.0 + width_m * index / (lane_count - 1)
            for index in range(lane_count)
        ]

    @staticmethod
    def clamp(value, min_value, max_value):
        return max(min(value, max_value), min_value)


def main(args=None):
    rclpy.init(args=args)
    node = TopdownTargetLocalizationMissionNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
