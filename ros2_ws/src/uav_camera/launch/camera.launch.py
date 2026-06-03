from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("uav_camera")

    gimbal_config = os.path.join(pkg_share, "config", "gimbal_camera.yaml")
    gimbal_cam = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="gimbal_camera",
        namespace="/uav/camera/front",
        output="screen",
        parameters=[gimbal_config],
        remappings=[
            ("image_raw", "image_raw"),
            ("camera_info", "camera_info"),
        ],
    )

    return LaunchDescription([
        gimbal_cam,
    ])
