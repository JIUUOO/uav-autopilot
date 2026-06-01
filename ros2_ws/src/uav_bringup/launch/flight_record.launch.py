#!/usr/bin/env python3

from datetime import datetime
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.actions import LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

'''
usage:

$ ros2 launch uav_bringup flight_record.launch.py \
  mission_package:=uav_bringup \
  mission_executable:=guided_takeoff_loiter \
  mission_node_name:=guided_takeoff_loiter \
  loiter_hold_sec:=5.0 \
  dry_run:=false \
  ntrip_user:=<ID> \
  ntrip_pass:=<PW> \
  ntrip_mountpoint:=SUWN-RTCM31
  
'''


def _is_true(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _make_bag_process(context, *_args, **_kwargs):
    if not _is_true(LaunchConfiguration("enable_rosbag").perform(context)):
        return []

    bag_root = os.path.expanduser(LaunchConfiguration("bag_root").perform(context))
    os.makedirs(bag_root, exist_ok=True)

    bag_name = LaunchConfiguration("bag_name").perform(context).strip()
    if not bag_name:
        bag_name = "flight_" + datetime.now().strftime("%Y%m%d_%H%M%S")

    output_path = os.path.join(bag_root, bag_name)
    cmd = ["ros2", "bag", "record", "-o", output_path]

    if _is_true(LaunchConfiguration("bag_record_all").perform(context)):
        cmd.append("-a")
    else:
        topics = LaunchConfiguration("bag_topics").perform(context).split()
        cmd.extend(topics)

    return [
        LogInfo(msg=f"[rosbag] output: {output_path}"),
        ExecuteProcess(cmd=cmd, output="screen"),
    ]


def generate_launch_description():
    mavros_default_fcu_url = "serial:///dev/ttyACM0:115200"
    mavros_default_gcs_url = "udp://:14555@127.0.0.1:14550"
    mission_default_port = "udpin:127.0.0.1:14550" # default USB port
    # Synced front camera config from the uav_camera package.
    front_camera_config = PathJoinSubstitution([
        FindPackageShare("uav_camera"),
        "config",
        "front_camera.yaml",
    ])
    
    default_bag_root = os.path.expanduser("~/bags")

    bag_topics_default = " ".join([
        "/mavros/state",
        "/mavros/extended_state",
        "/mavros/battery",
        "/mavros/imu/data",
        "/mavros/global_position/global",
        "/mavros/local_position/pose",
        "/mavros/altitude",
        "/uav/battery/voltage",
        "/uav/camera/front/image_raw",
        "/uav/camera/front/camera_info",
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
        DeclareLaunchArgument("enable_mavros", default_value="true"),
        DeclareLaunchArgument("mavros_fcu_url", default_value=mavros_default_fcu_url),
        DeclareLaunchArgument("mavros_gcs_url", default_value=mavros_default_gcs_url),
        DeclareLaunchArgument("tgt_system", default_value="1"),
        DeclareLaunchArgument("tgt_component", default_value="1"),
        DeclareLaunchArgument("enable_usb_cam", default_value="true"),
        DeclareLaunchArgument("enable_mission_node", default_value="true"),
        DeclareLaunchArgument("mission_package", default_value="uav_bringup"),  # ros package
        DeclareLaunchArgument("mission_executable", default_value="guided_takeoff_loiter"),  # ros executable
        DeclareLaunchArgument("mission_node_name", default_value="mission_node"),  # ros node
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
        DeclareLaunchArgument("bag_root", default_value=default_bag_root),
        DeclareLaunchArgument("bag_name", default_value=""),
        DeclareLaunchArgument("bag_topics", default_value=bag_topics_default),
        mavros_node,
        usb_cam_node,
        mission_node,
        battery_node,
        OpaqueFunction(function=_make_bag_process),
    ])
