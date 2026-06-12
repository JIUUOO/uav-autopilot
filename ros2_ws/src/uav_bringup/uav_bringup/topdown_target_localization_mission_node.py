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
    Detect, approach, center above a VLM-selected target, fix its RTK position, and return.

    Flight flow:
    PREPARE -> TAKEOFF -> LOITER_INITIAL_DETECTION
    -> no target: LAND
    -> target: GUIDED_XY_APPROACH -> LOITER_REASSESS -> GIMBAL_TOPDOWN
    -> GUIDED_CORRECTIONS/LOITER -> RTK_LOCALIZE -> RETURN_START -> LAND.
    Yaw is never commanded. The pre-takeoff yaw defines the fixed mission-front
    direction used to map body-frame XY recommendations into LOCAL_NED coordinates.
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

        self.latest_front_feedback = None
        self.latest_front_feedback_time = 0.0
        self.latest_front_report = None
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
        self.mission_front_yaw_deg = None
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
        self.declare_parameter("initial_detection_timeout_sec", 30.0)
        self.declare_parameter("approach_detection_timeout_sec", 30.0)
        self.declare_parameter("initial_loiter_settle_sec", 3.0)
        self.declare_parameter("approach_loiter_settle_sec", 2.0)
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
        self.declare_parameter("request_fresh_detection_after_approach", True)
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

        self.declare_parameter("completion_mode", "LAND")
        self.declare_parameter("abort_mode", "RTL")
        self.declare_parameter("mission_state_topic", "/uav/mission/state")

    def load_mission_params(self):
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
        self.initial_detection_timeout_sec = max(
            float(self.get_parameter("initial_detection_timeout_sec").value),
            0.0,
        )
        self.approach_detection_timeout_sec = max(
            float(self.get_parameter("approach_detection_timeout_sec").value),
            0.0,
        )
        self.initial_loiter_settle_sec = max(
            float(self.get_parameter("initial_loiter_settle_sec").value),
            0.0,
        )
        self.approach_loiter_settle_sec = max(
            float(self.get_parameter("approach_loiter_settle_sec").value),
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
        self.request_fresh_detection_after_approach = bool(
            self.get_parameter("request_fresh_detection_after_approach").value
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
        if not self.capture_mission_front_yaw():
            self.abort_mission("pre_takeoff_front_yaw_failed", set_abort_mode=False)
            return
        self.set_state("TAKEOFF")
        if not self.takeoff():
            self.abort_mission("takeoff_failed")
            return
        self.set_state("LOITER_INITIAL_DETECTION")
        if not self.flight_ops.set_mode("LOITER"):
            self.abort_mission("initial_loiter_failed")
            return
        if not self.wait_stable_loiter(self.initial_loiter_settle_sec):
            self.abort_mission("initial_loiter_settle_failed")
            return

        self.set_state("INITIAL_PERSON_DETECTION")
        approach_result = self.detect_and_approach_target()
        if approach_result == "NO_INITIAL_TARGET":
            if not self.land_without_target():
                self.abort_mission("no_target_land_failed")
                return
            self.finish_mission()
            return
        if approach_result != "TARGET_READY":
            self.abort_mission("target_approach_failed")
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

    def detect_and_approach_target(self):
        """Land on an initial miss; otherwise approach only through fresh VLM feedback."""

        initial_detection = True
        self.reset_front_observation()
        self.request_fresh_frame_selection("initial person detection")

        while rclpy.ok() and not self.stop_event.is_set():
            timeout_sec = (
                self.initial_detection_timeout_sec
                if initial_detection
                else self.approach_detection_timeout_sec
            )
            observation = self.wait_for_front_observation(timeout_sec)
            if observation == "NO_TARGET":
                if initial_detection:
                    self.get_logger().warn(
                        "Gemini did not detect a person after takeoff. Landing in place."
                    )
                    return "NO_INITIAL_TARGET"
                self.get_logger().error("Person was lost after approach started.")
                return "FAIL"
            if observation != "TARGET":
                return "FAIL"

            now = time.monotonic()
            if self.front_feedback_ready(now):
                self.get_logger().warn("Person is ready for top-down localization.")
                return "TARGET_READY"
            if not self.front_feedback_move_available(now):
                self.get_logger().error(
                    "Person detected, but no actionable approach movement was produced."
                )
                return "FAIL"
            self.set_state("GUIDED_XY_APPROACH")
            if not self.execute_front_feedback_move():
                return "FAIL"
            if not self.flight_ops.set_mode("LOITER"):
                return "FAIL"
            self.set_state("LOITER_REASSESS")
            if not self.wait_stable_loiter(self.approach_loiter_settle_sec):
                return "FAIL"

            initial_detection = False
            self.reset_front_observation()
            if not self.request_fresh_detection_after_approach:
                self.get_logger().error(
                    "Fresh detection after approach is required for closed-loop movement."
                )
                return "FAIL"
            self.request_fresh_frame_selection("person approach reassessment")

        return "FAIL"

    def wait_for_front_observation(self, timeout_sec):
        started_at = time.monotonic()
        while time.monotonic() - started_at <= timeout_sec:
            now = time.monotonic()
            self.spin_and_drain()
            if not self.safety_state_valid(now):
                return "FAIL"

            if self.latest_front_report is None:
                time.sleep(0.05)
                continue
            if not self.latest_front_report.parsed_ok:
                self.get_logger().error(
                    "Gemini person detection response was not valid: "
                    f"{self.latest_front_report.error_message}"
                )
                return "FAIL"
            if not self.latest_front_report.target_detected:
                return "NO_TARGET"
            if (
                self.front_feedback_fresh(now)
                and self.latest_front_feedback is not None
                and self.front_feedback_matches_report()
            ):
                if self.latest_front_feedback.has_target:
                    return "TARGET"
                self.get_logger().error(
                    "Gemini detected a person, but candidate feedback rejected it."
                )
                return "FAIL"
            time.sleep(0.05)

        self.get_logger().error("Timed out waiting for fresh person detection feedback.")
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

    def execute_front_feedback_move(self):
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
            return False

        self.total_front_moves += 1
        if not self.flight_ops.set_mode("GUIDED"):
            return False
        moved = self.local_ned.goto_offset(
            f"PERSON_APPROACH_{self.total_front_moves:02d}",
            target_north_m,
            target_east_m,
            self.config.altitude_m,
            acceptance_m=self.front_move_acceptance_m,
            timeout_sec=self.front_move_timeout_sec,
        )
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

    def land_without_target(self):
        self.set_state("NO_PERSON_LAND")
        self.publish_gimbal_pwm(self.return_gimbal_pwm, force=True)
        return self.flight_ops.set_mode("LAND")

    def abort_mission(self, reason, set_abort_mode=True):
        self.set_state("ABORT")
        self.get_logger().error(f"Mission abort: {reason}")
        self.topdown_active = False
        self.publish_gimbal_pwm(self.return_gimbal_pwm, force=True)
        if set_abort_mode and self.flight_ops is not None and self.abort_mode:
            self.flight_ops.set_mode(self.abort_mode)
        self.runtime.finish()

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

    def front_feedback_matches_report(self):
        if self.latest_front_report is None or self.latest_front_feedback is None:
            return False
        report_stamp = self.latest_front_report.header.stamp
        feedback_stamp = self.latest_front_feedback.header.stamp
        return (
            report_stamp.sec == feedback_stamp.sec
            and report_stamp.nanosec == feedback_stamp.nanosec
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
        if self.mission_front_yaw_deg is None:
            self.get_logger().error("Fixed mission-front yaw is unavailable for XY conversion.")
            return None
        delta_north_m, delta_east_m = self.local_ned.body_to_local_offset(
            forward_m,
            right_m,
            self.mission_front_yaw_deg,
        )
        return current[0] + delta_north_m, current[1] + delta_east_m

    def capture_mission_front_yaw(self):
        started_at = time.monotonic()
        while time.monotonic() - started_at < self.local_position_timeout_sec:
            self.spin_and_drain()
            if self.local_ned.yaw_fresh(time.monotonic()):
                self.mission_front_yaw_deg = self.local_ned.current_yaw_deg
                self.get_logger().warn(
                    f"Mission front fixed from pre-takeoff yaw: "
                    f"{self.mission_front_yaw_deg:.1f} deg"
                )
                return True
            time.sleep(0.05)

        self.get_logger().error("Failed to capture pre-takeoff yaw as mission front.")
        return False

    def wait_stable_loiter(self, duration_sec):
        """Wait in LOITER while continuing safety and telemetry checks."""

        started_at = time.monotonic()
        while time.monotonic() - started_at < duration_sec:
            now = time.monotonic()
            self.spin_and_drain()
            if not self.safety_state_valid(now):
                return False
            time.sleep(0.05)
        return True

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

    def reset_front_observation(self):
        self.latest_front_report = None
        self.latest_front_feedback = None
        self.latest_front_feedback_time = 0.0
        self.latest_front_detection = False
        self.latest_front_detection_time = 0.0

    def spin_and_drain(self):
        self.local_ned.spin_and_drain()
        if self.topdown_active:
            self.publish_gimbal_pwm(self.topdown_gimbal_pwm)

    def on_front_feedback(self, msg):
        self.latest_front_feedback = msg
        self.latest_front_feedback_time = time.monotonic()

    def on_gemini_report(self, msg):
        self.latest_front_report = msg
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
        self.get_logger().info(
            f"initial_detection_timeout  : {self.initial_detection_timeout_sec}"
        )
        self.get_logger().info(
            f"approach_detection_timeout : {self.approach_detection_timeout_sec}"
        )
        self.get_logger().info(f"initial_loiter_settle_sec  : {self.initial_loiter_settle_sec}")
        self.get_logger().info(f"approach_loiter_settle_sec : {self.approach_loiter_settle_sec}")
        self.get_logger().info(f"front_max_step_m           : {self.front_max_step_m}")
        self.get_logger().info(f"front_max_total_moves      : {self.front_max_total_moves}")
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
    def clamp(value, min_value, max_value):
        return max(min(value, max_value), min_value)


def main(args=None):
    rclpy.init(args=args)
    node = TopdownTargetLocalizationMissionNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
