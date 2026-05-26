#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from pymavlink import mavutil


class BatteryMonitorNode(Node):
    """
    Pixhawk battery monitor node.

    - Connects to Pixhawk through MAVLink
    - Requests SYS_STATUS at a fixed rate (Mavlink parameter)
    - Publishes battery voltage to ROS 2 topic
    - Prints battery voltage continuously
    """

    def __init__(self):
        super().__init__("battery_monitor_node")

        # parameter
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("print_hz", 1.0)
        self.declare_parameter("timeout_sec", 2.0)
        self.declare_parameter("low_voltage_threshold", 0.0)

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.print_hz = float(self.get_parameter("print_hz").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.low_voltage_threshold = float(
            self.get_parameter("low_voltage_threshold").value
        )

        self.voltage_pub = self.create_publisher(
            Float32,
            "/uav/battery/voltage",
            10,
        )

        self.get_logger().info("Battery monitor node started.")
        self.get_logger().info(f"port                 : {self.port}")
        self.get_logger().info(f"baudrate             : {self.baudrate}")
        self.get_logger().info(f"print_hz             : {self.print_hz}")
        self.get_logger().info(f"timeout_sec          : {self.timeout_sec}")
        self.get_logger().info(
            f"low_voltage_threshold: {self.low_voltage_threshold}"
        )

        self.conn = mavutil.mavlink_connection(
            self.port,
            baud=self.baudrate,
            autoreconnect=True,
        )

        self.get_logger().info("Waiting for MAVLink heartbeat...")
        self.conn.wait_heartbeat(timeout=10)

        self.get_logger().info(
            f"Heartbeat received: system={self.conn.target_system}, "
            f"component={self.conn.target_component}"
        )

        self.request_message_interval(
            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
            self.print_hz,
        )

        period = 1.0 / self.print_hz
        self.timer = self.create_timer(period, self.on_timer)

    def request_message_interval(self, msg_id: int, hz: float):
        interval_us = int(1_000_000 / hz)

        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )

    def on_timer(self):
        msg = self.conn.recv_match(
            type="SYS_STATUS",
            blocking=False,
        )

        if msg is None:
            self.get_logger().warn("No SYS_STATUS message received.")
            return

        data = msg.to_dict()
        voltage_mv = data.get("voltage_battery")

        if voltage_mv is None or voltage_mv <= 0 or voltage_mv >= 65535:
            self.get_logger().warn("battery_voltage = unknown")
            return

        voltage_v = voltage_mv / 1000.0

        voltage_msg = Float32()
        voltage_msg.data = float(voltage_v)
        self.voltage_pub.publish(voltage_msg)

        if (
            self.low_voltage_threshold > 0.0
            and voltage_v < self.low_voltage_threshold
        ):
            self.get_logger().warn(f"battery_voltage = {voltage_v:.2f} V LOW")
        else:
            self.get_logger().info(f"battery_voltage = {voltage_v:.2f} V")


def main(args=None):
    rclpy.init(args=args)

    node = BatteryMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Battery monitor stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
