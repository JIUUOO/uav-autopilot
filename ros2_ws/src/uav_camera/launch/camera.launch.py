from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("uav_camera")

    front_config = os.path.join(pkg_share, "config", "front_camera.yaml")
    front_cam = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="front_usb_cam",
        namespace="uav/camera/front",
        output="screen",
        parameters=[front_config],
        remappings=[
            ("image_raw", "image_raw"),
            ("camera_info", "camera_info"),
        ],
    )

    return LaunchDescription([
        front_cam,
    ])
