#!/usr/bin/env python3

from datetime import datetime
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


"""
Usage:

Baseline:
ros2 launch uav_bringup vision_experiment.launch.py \
  experiment_mode:=baseline \
  enable_selector:=false \
  enable_gemini:=true

Selector:
ros2 launch uav_bringup vision_experiment.launch.py \
  experiment_mode:=selector \
  enable_selector:=true \
  enable_gemini:=true
"""


def _is_true(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _gemini_image_topic(context) -> str:
    explicit_topic = LaunchConfiguration("gemini_image_topic").perform(context).strip()
    if explicit_topic:
        return explicit_topic

    experiment_mode = LaunchConfiguration("experiment_mode").perform(context).strip().lower()
    if experiment_mode == "baseline":
        return LaunchConfiguration("raw_image_topic").perform(context)
    if experiment_mode == "selector":
        return LaunchConfiguration("selected_image_topic").perform(context)

    raise RuntimeError("experiment_mode must be 'baseline' or 'selector'")


def _make_gemini_node(context, *_args, **_kwargs):
    if not _is_true(LaunchConfiguration("enable_gemini").perform(context)):
        return []

    image_topic = _gemini_image_topic(context)
    return [
        LogInfo(msg=f"[gemini] image_topic: {image_topic}"),
        Node(
            package="uav_vision",
            executable="gemini_frame_analyzer",
            name="gemini_frame_analyzer",
            output="screen",
            parameters=[{
                "image_topic": image_topic,
                "report_topic": LaunchConfiguration("gemini_report_topic"),
                "model": LaunchConfiguration("gemini_model"),
                "analysis_period_sec": LaunchConfiguration("gemini_period_sec"),
                "analysis_mode": LaunchConfiguration("gemini_analysis_mode"),
                "trigger_topic": LaunchConfiguration("gemini_trigger_topic"),
                "max_calls": LaunchConfiguration("gemini_max_calls"),
                "prompt_version": LaunchConfiguration("gemini_prompt_version"),
                "max_width": LaunchConfiguration("gemini_max_width"),
                "jpeg_quality": LaunchConfiguration("gemini_jpeg_quality"),
                "save_reports": LaunchConfiguration("save_gemini_reports"),
                "report_dir": LaunchConfiguration("gemini_report_dir"),
            }],
        ),
    ]


def _make_bag_process(context, *_args, **_kwargs):
    if not _is_true(LaunchConfiguration("enable_rosbag").perform(context)):
        return []

    bag_root = os.path.expanduser(LaunchConfiguration("bag_root").perform(context))
    os.makedirs(bag_root, exist_ok=True)

    bag_name = LaunchConfiguration("bag_name").perform(context).strip()
    if not bag_name:
        mode = LaunchConfiguration("experiment_mode").perform(context).strip().lower()
        bag_name = f"vision_{mode}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    output_path = os.path.join(bag_root, bag_name)
    cmd = ["ros2", "bag", "record", "-o", output_path]

    if _is_true(LaunchConfiguration("bag_record_all").perform(context)):
        cmd.append("-a")
    else:
        topics = LaunchConfiguration("bag_topics").perform(context).split()
        cmd.extend(topics)

    actions = [LogInfo(msg=f"[rosbag] output: {output_path}")]
    if _is_true(LaunchConfiguration("bag_record_all").perform(context)):
        actions.append(LogInfo(msg="[rosbag] topics: all"))
    else:
        actions.append(LogInfo(msg="[rosbag] topics: " + " ".join(topics)))

    actions.append(ExecuteProcess(cmd=cmd, output="screen"))
    return actions


def generate_launch_description():
    mavros_default_fcu_url = "serial:///dev/ttyACM0:115200"
    mavros_default_gcs_url = "udp://:14555@127.0.0.1:14550"
    mission_default_port = "udpin:127.0.0.1:14550"
    experiment_root = os.path.expanduser("~/uav_experiments")

    raw_image_topic = "/uav/camera/front/image_raw"
    unflipped_image_topic = "/uav/camera/front/image_raw_unflipped"
    selected_image_topic = "/uav/vision/selected_frame/image_raw"
    frame_quality_topic = "/uav/vision/frame_quality"
    gemini_report_topic = "/uav/vision/gemini_report"
    gemini_trigger_topic = "/uav/vision/analyze_trigger"
    candidate_tracks_topic = "/uav/vision/candidate_tracks"
    target_feedback_topic = "/uav/vision/target_feedback"
    person_position_estimate_topic = "/uav/vision/person_position_estimate"
    gimbal_pitch_target_topic = "/uav/gimbal/pitch_target_pwm"
    gimbal_state_topic = "/uav/gimbal/state"

    gimbal_camera_config = PathJoinSubstitution([
        FindPackageShare("uav_camera"),
        "config",
        "gimbal_camera.yaml",
    ])

    bag_topics_default = " ".join([
        raw_image_topic,
        "/uav/camera/front/camera_info",
        selected_image_topic,
        frame_quality_topic,
        gemini_report_topic,
        gemini_trigger_topic,
        candidate_tracks_topic,
        target_feedback_topic,
        person_position_estimate_topic,
        gimbal_pitch_target_topic,
        gimbal_state_topic,
        "/tf",  # Dynamic coordinate transforms for replaying vehicle/camera pose over time.
        "/tf_static",  # Static coordinate transforms such as base_link -> camera frame.
        "/mavros/state",
        "/mavros/extended_state",
        "/mavros/imu/data",
        "/mavros/imu/data_raw",  # Raw angular velocity and linear acceleration before filtering.
        "/mavros/imu/mag",  # Magnetometer data used to inspect compass/magnetic-field behavior.
        "/mavros/global_position/global",
        "/mavros/global_position/raw/fix",  # Raw GPS fix with covariance for GNSS quality analysis.
        "/mavros/global_position/rel_alt",  # Relative altitude from the home position.
        "/mavros/global_position/compass_hdg",  # Compass heading reported by the flight controller.
        "/mavros/local_position/pose",
        "/mavros/local_position/velocity_local",
        "/mavros/local_position/velocity_body",  # Body-frame velocity for motion-aware frame analysis.
        "/mavros/altitude",
        "/mavros/vfr_hud",  # HUD-style flight summary: speed, heading, throttle, altitude, climb.
        "/mavros/home_position/home",  # Pixhawk/MAVROS home position used as the mission reference.
        "/mavros/battery",
        "/uav/battery/voltage",
        "/rosout",
        "/parameter_events",
    ])

    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_mavros")),
        parameters=[
            PathJoinSubstitution([FindPackageShare("mavros"), "launch", "apm_pluginlists.yaml"]),
            PathJoinSubstitution([FindPackageShare("mavros"), "launch", "apm_config.yaml"]),
            {
                "fcu_url": LaunchConfiguration("mavros_fcu_url"),
                "gcs_url": LaunchConfiguration("mavros_gcs_url"),
                "tgt_system": LaunchConfiguration("tgt_system"),
                "tgt_component": LaunchConfiguration("tgt_component"),
            },
        ],
    )

    usb_cam_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="gimbal_camera",
        namespace="uav/camera/front",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_usb_cam")),
        parameters=[
            gimbal_camera_config,
            {"video_device": LaunchConfiguration("usb_cam_video_device")},
        ],
        remappings=[
            ("image_raw", "image_raw_unflipped"),
            ("camera_info", "camera_info"),
        ],
    )

    usb_cam_flip_node = Node(
        package="uav_camera",
        executable="image_flip",
        name="gimbal_camera_flip",
        namespace="uav/camera/front",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_usb_cam")),
        parameters=[{
            "input_topic": unflipped_image_topic,
            "output_topic": LaunchConfiguration("raw_image_topic"),
            "flip_mode": LaunchConfiguration("usb_cam_flip_mode"),
        }],
    )

    selector_node = Node(
        package="uav_vision",
        executable="frame_quality_selector",
        name="frame_quality_selector",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_selector")),
        parameters=[{
            "image_topic": LaunchConfiguration("raw_image_topic"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "selected_image_topic": LaunchConfiguration("selected_image_topic"),
            "quality_topic": LaunchConfiguration("frame_quality_topic"),
            "sample_hz": LaunchConfiguration("selector_sample_hz"),
            "selection_window_sec": LaunchConfiguration("selection_window_sec"),
            "score_width": LaunchConfiguration("selector_score_width"),
            "min_laplacian_var": LaunchConfiguration("min_laplacian_var"),
            "min_brightness": LaunchConfiguration("min_brightness"),
            "max_brightness": LaunchConfiguration("max_brightness"),
            "max_saturation_ratio": LaunchConfiguration("max_saturation_ratio"),
            "max_yaw_rate_rad_s": LaunchConfiguration("max_yaw_rate_rad_s"),
        }],
    )

    mission_node = Node(
        package=LaunchConfiguration("mission_package"),
        executable=LaunchConfiguration("mission_executable"),
        name=LaunchConfiguration("mission_node_name"),
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_mission_node")),
        parameters=[{
            "port": LaunchConfiguration("mission_port"),
            "dry_run": LaunchConfiguration("dry_run"),
            "altitude_m": LaunchConfiguration("mission_altitude_m"),
            "loiter_hold_sec": LaunchConfiguration("loiter_hold_sec"),
            "set_land_speed_params": LaunchConfiguration("set_land_speed_params"),
            "land_speed_cm_s": LaunchConfiguration("land_speed_cm_s"),
            "land_speed_high_cm_s": LaunchConfiguration("land_speed_high_cm_s"),
            "land_param_timeout_sec": LaunchConfiguration("land_param_timeout_sec"),
            "enable_ntrip": LaunchConfiguration("enable_ntrip"),
            "ntrip_host": LaunchConfiguration("ntrip_host"),
            "ntrip_port": LaunchConfiguration("ntrip_port"),
            "ntrip_mountpoint": LaunchConfiguration("ntrip_mountpoint"),
            "ntrip_user": LaunchConfiguration("ntrip_user"),
            "ntrip_pass": LaunchConfiguration("ntrip_pass"),
            "scout_radius_m": LaunchConfiguration("scout_radius_m"),
            "corner_offset_m": LaunchConfiguration("corner_offset_m"),
            "software_radius_m": LaunchConfiguration("software_radius_m"),
            "inspection_radius_m": LaunchConfiguration("inspection_radius_m"),
            "require_origin_before_takeoff": LaunchConfiguration("require_origin_before_takeoff"),
            "origin_sample_count": LaunchConfiguration("origin_sample_count"),
            "waypoint_acceptance_m": LaunchConfiguration("waypoint_acceptance_m"),
            "waypoint_timeout_sec": LaunchConfiguration("waypoint_timeout_sec"),
            "setpoint_hz": LaunchConfiguration("setpoint_hz"),
            "local_position_timeout_sec": LaunchConfiguration("local_position_timeout_sec"),
            "waypoint_hold_sec": LaunchConfiguration("waypoint_hold_sec"),
            "route_pattern": LaunchConfiguration("route_pattern"),
            "yaw_angles_deg": LaunchConfiguration("yaw_angles_deg"),
            "yaw_hold_sec": LaunchConfiguration("yaw_hold_sec"),
            "yaw_speed_deg_s": LaunchConfiguration("yaw_speed_deg_s"),
            "yaw_acceptance_deg": LaunchConfiguration("yaw_acceptance_deg"),
            "yaw_timeout_sec": LaunchConfiguration("yaw_timeout_sec"),
            "route_mode": LaunchConfiguration("route_mode"),
            "square_side_m": LaunchConfiguration("square_side_m"),
            "square_yaws_deg": LaunchConfiguration("square_yaws_deg"),
            "square_origin_mode": LaunchConfiguration("square_origin_mode"),
            "front_length_m": LaunchConfiguration("front_length_m"),
            "front_width_m": LaunchConfiguration("front_width_m"),
            "front_lane_count": LaunchConfiguration("front_lane_count"),
            "front_points_per_lane": LaunchConfiguration("front_points_per_lane"),
            "use_current_yaw_as_front": LaunchConfiguration("use_current_yaw_as_front"),
            "front_yaw_deg": LaunchConfiguration("front_yaw_deg"),
            "route_hold_sec": LaunchConfiguration("front_route_hold_sec"),
            "target_feedback_topic": LaunchConfiguration("target_feedback_topic"),
            "target_feedback_timeout_sec": LaunchConfiguration("target_feedback_timeout_sec"),
            "stop_on_ready_to_inspect": LaunchConfiguration("stop_on_ready_to_inspect"),
            "enable_target_feedback_movement": LaunchConfiguration("enable_target_feedback_movement"),
            "max_feedback_step_m": LaunchConfiguration("max_feedback_step_m"),
            "feedback_move_acceptance_m": LaunchConfiguration("feedback_move_acceptance_m"),
            "feedback_move_timeout_sec": LaunchConfiguration("feedback_move_timeout_sec"),
            "feedback_move_cooldown_sec": LaunchConfiguration("feedback_move_cooldown_sec"),
            "max_feedback_moves_per_waypoint": LaunchConfiguration("max_feedback_moves_per_waypoint"),
            "max_total_feedback_moves": LaunchConfiguration("max_total_feedback_moves"),
            "gemini_report_topic": LaunchConfiguration("gemini_report_topic"),
            "enable_low_altitude_inspection": LaunchConfiguration("enable_low_altitude_inspection"),
            "inspect_altitude_m": LaunchConfiguration("inspect_altitude_m"),
            "min_safe_altitude_m": LaunchConfiguration("min_safe_altitude_m"),
            "inspection_hold_sec": LaunchConfiguration("inspection_hold_sec"),
            "inspection_max_count": LaunchConfiguration("inspection_max_count"),
            "disable_inspection_for_person": LaunchConfiguration("disable_inspection_for_person"),
            "person_safe_altitude_m": LaunchConfiguration("person_safe_altitude_m"),
            "finish_mode": LaunchConfiguration("finish_mode"),
            "abort_mode": LaunchConfiguration("abort_mode"),
        }],
    )

    battery_node = Node(
        package="uav_bringup",
        executable="battery_monitor",
        name="battery_monitor",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_battery_monitor")),
        parameters=[{
            "port": LaunchConfiguration("battery_monitor_port"),
            "baudrate": LaunchConfiguration("battery_monitor_baudrate"),
            "print_hz": LaunchConfiguration("battery_monitor_print_hz"),
            "low_voltage_threshold": LaunchConfiguration("battery_low_voltage_threshold"),
        }],
    )

    gimbal_node = Node(
        package="uav_gimbal",
        executable="gimbal_pitch_controller",
        name="gimbal_pitch_controller",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_gimbal")),
        parameters=[{
            "dry_run": LaunchConfiguration("gimbal_dry_run"),
            "command_service": LaunchConfiguration("gimbal_command_service"),
            "gemini_report_topic": LaunchConfiguration("gemini_report_topic"),
            "pitch_target_topic": LaunchConfiguration("gimbal_pitch_target_topic"),
            "state_topic": LaunchConfiguration("gimbal_state_topic"),
            "servo_channel": LaunchConfiguration("gimbal_servo_channel"),
            "pwm_min": LaunchConfiguration("gimbal_pwm_min"),
            "pwm_neutral": LaunchConfiguration("gimbal_pwm_neutral"),
            "pwm_max": LaunchConfiguration("gimbal_pwm_max"),
            "invert_pwm": LaunchConfiguration("gimbal_invert_pwm"),
            "preset_far_pwm": LaunchConfiguration("gimbal_preset_far_pwm"),
            "preset_near_pwm": LaunchConfiguration("gimbal_preset_near_pwm"),
            "command_hz": LaunchConfiguration("gimbal_command_hz"),
            "max_rate_pwm_s": LaunchConfiguration("gimbal_max_rate_pwm_s"),
            "command_timeout_sec": LaunchConfiguration("gimbal_command_timeout_sec"),
            "timeout_to_neutral": LaunchConfiguration("gimbal_timeout_to_neutral"),
            "confidence_threshold": LaunchConfiguration("gimbal_confidence_threshold"),
        }],
    )

    gimbal_scan_node = Node(
        package="uav_gimbal",
        executable="gimbal_scan_fsm",
        name="gimbal_scan_fsm",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_gimbal_scan")),
        parameters=[{
            "pitch_target_topic": LaunchConfiguration("gimbal_pitch_target_topic"),
            "trigger_topic": LaunchConfiguration("gemini_trigger_topic"),
            "preset_far_pwm": LaunchConfiguration("gimbal_preset_far_pwm"),
            "preset_near_pwm": LaunchConfiguration("gimbal_preset_near_pwm"),
            "scan_sequence": LaunchConfiguration("gimbal_scan_sequence"),
            "settle_sec": LaunchConfiguration("gimbal_scan_settle_sec"),
            "post_trigger_sec": LaunchConfiguration("gimbal_scan_post_trigger_sec"),
            "scan_count": LaunchConfiguration("gimbal_scan_count"),
            "auto_start": LaunchConfiguration("gimbal_scan_auto_start"),
        }],
    )

    candidate_manager_node = Node(
        package="uav_vision",
        executable="candidate_manager",
        name="candidate_manager",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_candidate_manager")),
        parameters=[{
            "gemini_report_topic": LaunchConfiguration("gemini_report_topic"),
            "tracks_topic": LaunchConfiguration("candidate_tracks_topic"),
            "min_confidence": LaunchConfiguration("candidate_min_confidence"),
            "match_center_distance": LaunchConfiguration("candidate_match_center_distance"),
            "max_track_age_sec": LaunchConfiguration("candidate_max_track_age_sec"),
            "publish_empty": LaunchConfiguration("candidate_publish_empty"),
        }],
    )

    target_feedback_node = Node(
        package="uav_vision",
        executable="target_feedback",
        name="target_feedback",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_target_feedback")),
        parameters=[{
            "tracks_topic": LaunchConfiguration("candidate_tracks_topic"),
            "feedback_topic": LaunchConfiguration("target_feedback_topic"),
            "min_priority_score": LaunchConfiguration("feedback_min_priority_score"),
            "center_x_min": LaunchConfiguration("feedback_center_x_min"),
            "center_x_max": LaunchConfiguration("feedback_center_x_max"),
            "center_y_min": LaunchConfiguration("feedback_center_y_min"),
            "center_y_max": LaunchConfiguration("feedback_center_y_max"),
            "side_step_m": LaunchConfiguration("feedback_side_step_m"),
            "forward_step_m": LaunchConfiguration("feedback_forward_step_m"),
            "approach_distance_bucket": LaunchConfiguration("feedback_approach_distance_bucket"),
            "inspect_distance_bucket": LaunchConfiguration("feedback_inspect_distance_bucket"),
            "require_centered_for_inspect": LaunchConfiguration("feedback_require_centered_for_inspect"),
        }],
    )

    person_position_estimator_node = Node(
        package="uav_vision",
        executable="rtk_person_position_estimator",
        name="rtk_person_position_estimator",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_person_position_estimator")),
        parameters=[{
            "target_feedback_topic": LaunchConfiguration("target_feedback_topic"),
            "gps_topic": LaunchConfiguration("person_estimator_gps_topic"),
            "heading_topic": LaunchConfiguration("person_estimator_heading_topic"),
            "estimate_topic": LaunchConfiguration("person_position_estimate_topic"),
            "feedback_timeout_sec": LaunchConfiguration("person_estimator_feedback_timeout_sec"),
            "gps_timeout_sec": LaunchConfiguration("person_estimator_gps_timeout_sec"),
            "heading_timeout_sec": LaunchConfiguration("person_estimator_heading_timeout_sec"),
            "min_priority_score": LaunchConfiguration("person_estimator_min_priority_score"),
            "require_ready_to_inspect_for_estimate": LaunchConfiguration("person_estimator_require_ready_to_inspect"),
            "near_distance_m": LaunchConfiguration("person_estimator_near_distance_m"),
            "far_distance_m": LaunchConfiguration("person_estimator_far_distance_m"),
            "unknown_distance_m": LaunchConfiguration("person_estimator_unknown_distance_m"),
            "lateral_error_gain_m": LaunchConfiguration("person_estimator_lateral_error_gain_m"),
            "max_lateral_offset_m": LaunchConfiguration("person_estimator_max_lateral_offset_m"),
            "horizontal_error_margin_m": LaunchConfiguration("person_estimator_horizontal_error_margin_m"),
            "assumed_target_below_m": LaunchConfiguration("person_estimator_assumed_target_below_m"),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("experiment_mode", default_value="selector"),
        DeclareLaunchArgument("enable_mavros", default_value="true"),
        DeclareLaunchArgument("mavros_fcu_url", default_value=mavros_default_fcu_url),
        DeclareLaunchArgument("mavros_gcs_url", default_value=mavros_default_gcs_url),
        DeclareLaunchArgument("tgt_system", default_value="1"),
        DeclareLaunchArgument("tgt_component", default_value="1"),
        DeclareLaunchArgument("enable_usb_cam", default_value="true"),
        DeclareLaunchArgument(
            "usb_cam_video_device",
            default_value="/dev/video0",
        ),
        DeclareLaunchArgument("usb_cam_flip_mode", default_value="rotate_180"),
        DeclareLaunchArgument("enable_selector", default_value="true"),
        DeclareLaunchArgument("enable_gemini", default_value="true"),
        DeclareLaunchArgument("enable_candidate_manager", default_value="false"),
        DeclareLaunchArgument("enable_target_feedback", default_value="false"),
        DeclareLaunchArgument("enable_person_position_estimator", default_value="false"),
        DeclareLaunchArgument("enable_gimbal", default_value="false"),
        DeclareLaunchArgument("enable_gimbal_scan", default_value="false"),
        DeclareLaunchArgument("raw_image_topic", default_value=raw_image_topic),
        DeclareLaunchArgument("selected_image_topic", default_value=selected_image_topic),
        DeclareLaunchArgument("frame_quality_topic", default_value=frame_quality_topic),
        DeclareLaunchArgument("imu_topic", default_value="/mavros/imu/data"),
        DeclareLaunchArgument("selector_sample_hz", default_value="5.0"),
        DeclareLaunchArgument("selection_window_sec", default_value="5.0"),
        DeclareLaunchArgument("selector_score_width", default_value="160"),
        DeclareLaunchArgument("min_laplacian_var", default_value="30.0"),
        DeclareLaunchArgument("min_brightness", default_value="30.0"),
        DeclareLaunchArgument("max_brightness", default_value="225.0"),
        DeclareLaunchArgument("max_saturation_ratio", default_value="0.35"),
        DeclareLaunchArgument("max_yaw_rate_rad_s", default_value="0.8"),
        DeclareLaunchArgument("gemini_image_topic", default_value=""),
        DeclareLaunchArgument("gemini_report_topic", default_value=gemini_report_topic),
        DeclareLaunchArgument("gemini_trigger_topic", default_value=gemini_trigger_topic),
        DeclareLaunchArgument("gemini_model", default_value="gemini-2.5-flash"),
        DeclareLaunchArgument("gemini_analysis_mode", default_value="periodic"),
        DeclareLaunchArgument("gemini_period_sec", default_value="5.0"),
        DeclareLaunchArgument("gemini_max_calls", default_value="0"),
        DeclareLaunchArgument("gemini_prompt_version", default_value="person_bbox_v1"),
        DeclareLaunchArgument("gemini_max_width", default_value="640"),
        DeclareLaunchArgument("gemini_jpeg_quality", default_value="70"),
        DeclareLaunchArgument("save_gemini_reports", default_value="true"),
        DeclareLaunchArgument(
            "gemini_report_dir",
            default_value=os.path.join(experiment_root, "gemini_reports"),
        ),
        DeclareLaunchArgument("candidate_tracks_topic", default_value=candidate_tracks_topic),
        DeclareLaunchArgument("candidate_min_confidence", default_value="0.30"),
        DeclareLaunchArgument("candidate_match_center_distance", default_value="0.15"),
        DeclareLaunchArgument("candidate_max_track_age_sec", default_value="30.0"),
        DeclareLaunchArgument("candidate_publish_empty", default_value="true"),
        DeclareLaunchArgument("target_feedback_topic", default_value=target_feedback_topic),
        DeclareLaunchArgument("feedback_min_priority_score", default_value="0.50"),
        DeclareLaunchArgument("feedback_center_x_min", default_value="0.40"),
        DeclareLaunchArgument("feedback_center_x_max", default_value="0.60"),
        DeclareLaunchArgument("feedback_center_y_min", default_value="0.35"),
        DeclareLaunchArgument("feedback_center_y_max", default_value="0.70"),
        DeclareLaunchArgument("feedback_side_step_m", default_value="0.30"),
        DeclareLaunchArgument("feedback_forward_step_m", default_value="0.50"),
        DeclareLaunchArgument("feedback_approach_distance_bucket", default_value="far"),
        DeclareLaunchArgument("feedback_inspect_distance_bucket", default_value="near"),
        DeclareLaunchArgument("feedback_require_centered_for_inspect", default_value="true"),
        DeclareLaunchArgument("person_position_estimate_topic", default_value=person_position_estimate_topic),
        DeclareLaunchArgument("person_estimator_gps_topic", default_value="/mavros/global_position/global"),
        DeclareLaunchArgument("person_estimator_heading_topic", default_value="/mavros/global_position/compass_hdg"),
        DeclareLaunchArgument("person_estimator_feedback_timeout_sec", default_value="5.0"),
        DeclareLaunchArgument("person_estimator_gps_timeout_sec", default_value="3.0"),
        DeclareLaunchArgument("person_estimator_heading_timeout_sec", default_value="3.0"),
        DeclareLaunchArgument("person_estimator_min_priority_score", default_value="0.50"),
        DeclareLaunchArgument("person_estimator_require_ready_to_inspect", default_value="false"),
        DeclareLaunchArgument("person_estimator_near_distance_m", default_value="2.0"),
        DeclareLaunchArgument("person_estimator_far_distance_m", default_value="5.0"),
        DeclareLaunchArgument("person_estimator_unknown_distance_m", default_value="3.0"),
        DeclareLaunchArgument("person_estimator_lateral_error_gain_m", default_value="4.0"),
        DeclareLaunchArgument("person_estimator_max_lateral_offset_m", default_value="2.0"),
        DeclareLaunchArgument("person_estimator_horizontal_error_margin_m", default_value="2.0"),
        DeclareLaunchArgument("person_estimator_assumed_target_below_m", default_value="0.0"),
        DeclareLaunchArgument("gimbal_dry_run", default_value="true"),
        DeclareLaunchArgument("gimbal_command_service", default_value="/mavros/cmd/command"),
        DeclareLaunchArgument("gimbal_pitch_target_topic", default_value=gimbal_pitch_target_topic),
        DeclareLaunchArgument("gimbal_state_topic", default_value=gimbal_state_topic),
        DeclareLaunchArgument("gimbal_servo_channel", default_value="7"),
        DeclareLaunchArgument("gimbal_pwm_min", default_value="1530"),
        DeclareLaunchArgument("gimbal_pwm_neutral", default_value="1560"),
        DeclareLaunchArgument("gimbal_pwm_max", default_value="1590"),
        DeclareLaunchArgument("gimbal_invert_pwm", default_value="true"),
        DeclareLaunchArgument("gimbal_preset_far_pwm", default_value="1560"),
        DeclareLaunchArgument("gimbal_preset_near_pwm", default_value="1530"),
        DeclareLaunchArgument("gimbal_command_hz", default_value="5.0"),
        DeclareLaunchArgument("gimbal_max_rate_pwm_s", default_value="100.0"),
        DeclareLaunchArgument("gimbal_command_timeout_sec", default_value="2.0"),
        DeclareLaunchArgument("gimbal_timeout_to_neutral", default_value="true"),
        DeclareLaunchArgument("gimbal_confidence_threshold", default_value="0.75"),
        DeclareLaunchArgument("gimbal_scan_sequence", default_value="PRESET_FAR,PRESET_NEAR"),
        DeclareLaunchArgument("gimbal_scan_settle_sec", default_value="1.0"),
        DeclareLaunchArgument("gimbal_scan_post_trigger_sec", default_value="0.2"),
        DeclareLaunchArgument("gimbal_scan_count", default_value="1"),
        DeclareLaunchArgument("gimbal_scan_auto_start", default_value="true"),
        DeclareLaunchArgument("enable_mission_node", default_value="false"),
        DeclareLaunchArgument("mission_package", default_value="uav_bringup"),
        DeclareLaunchArgument("mission_executable", default_value="guided_takeoff_loiter"),
        DeclareLaunchArgument("mission_node_name", default_value="mission_node"),
        DeclareLaunchArgument("mission_port", default_value=mission_default_port),
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("mission_altitude_m", default_value="3.5"),
        DeclareLaunchArgument("loiter_hold_sec", default_value="0.0"),
        DeclareLaunchArgument("set_land_speed_params", default_value="true"),
        DeclareLaunchArgument("land_speed_cm_s", default_value="30.0"),
        DeclareLaunchArgument("land_speed_high_cm_s", default_value="30.0"),
        DeclareLaunchArgument("land_param_timeout_sec", default_value="5.0"),
        DeclareLaunchArgument("scout_radius_m", default_value="5.0"),
        DeclareLaunchArgument("corner_offset_m", default_value="5.0"),
        DeclareLaunchArgument("software_radius_m", default_value="8.0"),
        DeclareLaunchArgument("inspection_radius_m", default_value="7.2"),
        DeclareLaunchArgument("require_origin_before_takeoff", default_value="true"),
        DeclareLaunchArgument("origin_sample_count", default_value="5"),
        DeclareLaunchArgument("waypoint_acceptance_m", default_value="0.8"),
        DeclareLaunchArgument("waypoint_timeout_sec", default_value="30.0"),
        DeclareLaunchArgument("setpoint_hz", default_value="2.0"),
        DeclareLaunchArgument("local_position_timeout_sec", default_value="5.0"),
        DeclareLaunchArgument("waypoint_hold_sec", default_value="5.0"),
        DeclareLaunchArgument("route_pattern", default_value="N,NE,E,SE,S,SW,W,NW"),
        DeclareLaunchArgument("yaw_angles_deg", default_value="0,45,90,135,180,225,270,315"),
        DeclareLaunchArgument("yaw_hold_sec", default_value="2.0"),
        DeclareLaunchArgument("yaw_speed_deg_s", default_value="20.0"),
        DeclareLaunchArgument("yaw_acceptance_deg", default_value="8.0"),
        DeclareLaunchArgument("yaw_timeout_sec", default_value="8.0"),
        DeclareLaunchArgument("route_mode", default_value="waypoints"),
        DeclareLaunchArgument("square_side_m", default_value="2.0"),
        DeclareLaunchArgument("square_yaws_deg", default_value="0,90,180,270"),
        DeclareLaunchArgument("square_origin_mode", default_value="center"),
        DeclareLaunchArgument("front_length_m", default_value="8.0"),
        DeclareLaunchArgument("front_width_m", default_value="4.0"),
        DeclareLaunchArgument("front_lane_count", default_value="3"),
        DeclareLaunchArgument("front_points_per_lane", default_value="3"),
        DeclareLaunchArgument("use_current_yaw_as_front", default_value="true"),
        DeclareLaunchArgument("front_yaw_deg", default_value="0.0"),
        DeclareLaunchArgument("front_route_hold_sec", default_value="2.0"),
        DeclareLaunchArgument("target_feedback_timeout_sec", default_value="5.0"),
        DeclareLaunchArgument("stop_on_ready_to_inspect", default_value="false"),
        DeclareLaunchArgument("enable_target_feedback_movement", default_value="false"),
        DeclareLaunchArgument("max_feedback_step_m", default_value="0.5"),
        DeclareLaunchArgument("feedback_move_acceptance_m", default_value="0.4"),
        DeclareLaunchArgument("feedback_move_timeout_sec", default_value="10.0"),
        DeclareLaunchArgument("feedback_move_cooldown_sec", default_value="2.0"),
        DeclareLaunchArgument("max_feedback_moves_per_waypoint", default_value="1"),
        DeclareLaunchArgument("max_total_feedback_moves", default_value="5"),
        DeclareLaunchArgument("enable_low_altitude_inspection", default_value="false"),
        DeclareLaunchArgument("inspect_altitude_m", default_value="2.0"),
        DeclareLaunchArgument("min_safe_altitude_m", default_value="2.0"),
        DeclareLaunchArgument("inspection_hold_sec", default_value="5.0"),
        DeclareLaunchArgument("inspection_max_count", default_value="1"),
        DeclareLaunchArgument("disable_inspection_for_person", default_value="true"),
        DeclareLaunchArgument("person_safe_altitude_m", default_value="3.5"),
        DeclareLaunchArgument("finish_mode", default_value="LAND"),
        DeclareLaunchArgument("abort_mode", default_value="RTL"),
        DeclareLaunchArgument("enable_ntrip", default_value="true"),
        DeclareLaunchArgument("ntrip_host", default_value="www.gnssdata.or.kr"),
        DeclareLaunchArgument("ntrip_port", default_value="2101"),
        DeclareLaunchArgument("ntrip_mountpoint", default_value="SUWN-RTCM31"),
        DeclareLaunchArgument("ntrip_user", default_value=os.getenv("NTRIP_USER", "")),
        DeclareLaunchArgument("ntrip_pass", default_value=os.getenv("NTRIP_PASS", "gnss")),
        DeclareLaunchArgument("enable_battery_monitor", default_value="false"),
        DeclareLaunchArgument("battery_monitor_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("battery_monitor_baudrate", default_value="115200"),
        DeclareLaunchArgument("battery_monitor_print_hz", default_value="1.0"),
        DeclareLaunchArgument("battery_low_voltage_threshold", default_value="0.0"),
        DeclareLaunchArgument("enable_rosbag", default_value="true"),
        DeclareLaunchArgument("bag_record_all", default_value="false"),
        DeclareLaunchArgument("bag_root", default_value=os.path.join(experiment_root, "bags")),
        DeclareLaunchArgument("bag_name", default_value=""),
        DeclareLaunchArgument("bag_topics", default_value=bag_topics_default),
        mavros_node,
        usb_cam_node,
        usb_cam_flip_node,
        selector_node,
        mission_node,
        battery_node,
        gimbal_node,
        gimbal_scan_node,
        candidate_manager_node,
        target_feedback_node,
        person_position_estimator_node,
        OpaqueFunction(function=_make_gemini_node),
        OpaqueFunction(function=_make_bag_process),
    ])
