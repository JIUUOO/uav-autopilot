import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class SimLidarSliceNode(Node):
    def __init__(self):
        super().__init__("sim_lidar_slice_node")

        self.declare_parameter("publish_hz", 4.0)
        self.declare_parameter("frame_id", "lidar_vertical")
        self.declare_parameter("angle_min_deg", -170.0)
        self.declare_parameter("angle_max_deg", -10.0)
        self.declare_parameter("range_min_m", 0.10)
        self.declare_parameter("range_max_m", 30.0)

        self.scan_pub = self.create_publisher(LaserScan, "/sim/lidar/scan", 10)

        hz = float(self.get_parameter("publish_hz").value)
        self.timer = self.create_timer(1.0 / hz, self.publish_scan)

        self.t = 0.0

        self.get_logger().info("sim_lidar_slice_node started")

    def publish_scan(self):
        frame_id = str(self.get_parameter("frame_id").value)
        angle_min_deg = float(self.get_parameter("angle_min_deg").value)
        angle_max_deg = float(self.get_parameter("angle_max_deg").value)
        range_min = float(self.get_parameter("range_min_m").value)
        range_max = float(self.get_parameter("range_max_m").value)

        angle_min = math.radians(angle_min_deg)
        angle_max = math.radians(angle_max_deg)

        sample_count = 240
        angle_increment = (angle_max - angle_min) / max(sample_count - 1, 1)

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = frame_id

        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / float(self.get_parameter("publish_hz").value)

        scan.range_min = range_min
        scan.range_max = range_max

        ranges = []

        for i in range(sample_count):
            a = angle_min + i * angle_increment

            # Simple synthetic terrain-like slice.
            # This is not physics; it is only a dummy vertical scan signal.
            base = 2.0 + 0.5 * math.sin(3.0 * a + self.t)
            bump = 0.4 * math.exp(-((a + math.pi / 2.0) ** 2) / 0.2)

            r = base + bump

            if r < range_min or r > range_max:
                ranges.append(float("inf"))
            else:
                ranges.append(float(r))

        scan.ranges = ranges
        scan.intensities = [1.0] * sample_count

        self.scan_pub.publish(scan)

        self.t += 0.05


def main(args=None):
    rclpy.init(args=args)
    node = SimLidarSliceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()