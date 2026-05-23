import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState, Imu


class SimVehicleStateNode(Node):
    def __init__(self):
        super().__init__("sim_vehicle_state_node")

        self.declare_parameter("publish_hz", 10.0)

        self.pose_pub = self.create_publisher(PoseStamped, "/sim/vehicle/pose", 10)
        self.twist_pub = self.create_publisher(TwistStamped, "/sim/vehicle/twist", 10)
        self.imu_pub = self.create_publisher(Imu, "/sim/vehicle/imu", 10)
        self.battery_pub = self.create_publisher(BatteryState, "/sim/vehicle/battery", 10)

        self.t = 0.0

        hz = float(self.get_parameter("publish_hz").value)
        self.timer = self.create_timer(1.0 / hz, self.publish_state)

        self.get_logger().info("sim_vehicle_state_node started")

    def make_header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        return header

    def publish_state(self):
        header = self.make_header()

        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = 0.5 * math.sin(self.t)
        pose.pose.position.y = 0.5 * math.cos(self.t)
        pose.pose.position.z = 1.5

        # Dummy orientation: identity quaternion.
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        twist = TwistStamped()
        twist.header = header
        twist.twist.linear.x = 0.1 * math.cos(self.t)
        twist.twist.linear.y = -0.1 * math.sin(self.t)
        twist.twist.linear.z = 0.0
        twist.twist.angular.z = 0.05

        imu = Imu()
        imu.header = header
        imu.orientation = pose.pose.orientation
        imu.angular_velocity.z = 0.05
        imu.linear_acceleration.z = 9.81

        battery = BatteryState()
        battery.header = header
        battery.voltage = 11.7
        battery.percentage = 0.75

        self.pose_pub.publish(pose)
        self.twist_pub.publish(twist)
        self.imu_pub.publish(imu)
        self.battery_pub.publish(battery)

        self.t += 0.02


def main(args=None):
    rclpy.init(args=args)
    node = SimVehicleStateNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()