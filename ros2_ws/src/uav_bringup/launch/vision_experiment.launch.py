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
    selected_image_topic = "/uav/vision/selected_frame/image_raw"
    frame_quality_topic = "/uav/vision/frame_quality"
    gemini_report_topic = "/uav/vision/gemini_report"

    front_camera_config = PathJoinSubstitution([
        FindPackageShare("uav_camera"),
        "config",
        "front_camera.yaml",
    ])

    bag_topics_default = " ".join([
        raw_image_topic,
        "/uav/camera/front/camera_info",
        selected_image_topic,
        frame_quality_topic,
        gemini_report_topic,
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
        name="front_usb_cam",
        namespace="uav/camera/front",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_usb_cam")),
        parameters=[front_camera_config],
        remappings=[
            ("image_raw", "image_raw"),
            ("camera_info", "camera_info"),
        ],
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
            "loiter_hold_sec": LaunchConfiguration("loiter_hold_sec"),
            "enable_ntrip": LaunchConfiguration("enable_ntrip"),
            "ntrip_host": LaunchConfiguration("ntrip_host"),
            "ntrip_port": LaunchConfiguration("ntrip_port"),
            "ntrip_mountpoint": LaunchConfiguration("ntrip_mountpoint"),
            "ntrip_user": LaunchConfiguration("ntrip_user"),
            "ntrip_pass": LaunchConfiguration("ntrip_pass"),
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

    return LaunchDescription([
        DeclareLaunchArgument("experiment_mode", default_value="selector"),
        DeclareLaunchArgument("enable_mavros", default_value="true"),
        DeclareLaunchArgument("mavros_fcu_url", default_value=mavros_default_fcu_url),
        DeclareLaunchArgument("mavros_gcs_url", default_value=mavros_default_gcs_url),
        DeclareLaunchArgument("tgt_system", default_value="1"),
        DeclareLaunchArgument("tgt_component", default_value="1"),
        DeclareLaunchArgument("enable_usb_cam", default_value="true"),
        DeclareLaunchArgument("enable_selector", default_value="true"),
        DeclareLaunchArgument("enable_gemini", default_value="true"),
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
        DeclareLaunchArgument("gemini_model", default_value="gemini-2.5-flash"),
        DeclareLaunchArgument("gemini_period_sec", default_value="5.0"),
        DeclareLaunchArgument("gemini_max_width", default_value="640"),
        DeclareLaunchArgument("gemini_jpeg_quality", default_value="70"),
        DeclareLaunchArgument("save_gemini_reports", default_value="true"),
        DeclareLaunchArgument(
            "gemini_report_dir",
            default_value=os.path.join(experiment_root, "gemini_reports"),
        ),
        DeclareLaunchArgument("enable_mission_node", default_value="false"),
        DeclareLaunchArgument("mission_package", default_value="uav_bringup"),
        DeclareLaunchArgument("mission_executable", default_value="guided_takeoff_loiter"),
        DeclareLaunchArgument("mission_node_name", default_value="mission_node"),
        DeclareLaunchArgument("mission_port", default_value=mission_default_port),
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("loiter_hold_sec", default_value="0.0"),
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
        selector_node,
        mission_node,
        battery_node,
        OpaqueFunction(function=_make_gemini_node),
        OpaqueFunction(function=_make_bag_process),
    ])
