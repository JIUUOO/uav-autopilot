from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="uav_sim",
                executable="sim_vehicle_state_node",
                name="sim_vehicle_state_node",
                output="screen",
                parameters=[
                    {
                        "publish_hz": 10.0,
                    }
                ],
            ),
            Node(
                package="uav_sim",
                executable="sim_lidar_slice_node",
                name="sim_lidar_slice_node",
                output="screen",
                parameters=[
                    {
                        "publish_hz": 4.0,
                        "frame_id": "lidar_vertical",
                        "angle_min_deg": -170.0,
                        "angle_max_deg": -10.0,
                        "range_min_m": 0.10,
                        "range_max_m": 30.0,
                    }
                ],
            ),
            Node(
                package="uav_sim",
                executable="sim_detection_event_node",
                name="sim_detection_event_node",
                output="screen",
                parameters=[
                    {
                        "publish_hz": 0.1,
                    }
                ],
            ),
        ]
    )