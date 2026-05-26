#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from pymavlink import mavutil


class DisarmNode(Node):
    """
    Pixhawk disarm command node

    - Connect to Pixhawk through MAVLink
    - Wait for heartbeat
    - Send disarm command
    - Wait for COMMAND_ACK
    - Exit
    """

    def __init__(self):
        super().__init__("disarm_node")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("timeout_sec", 10.0)

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)

        self.get_logger().info("Disarm node started.")
        self.get_logger().info(f"port       : {self.port}")
        self.get_logger().info(f"baudrate   : {self.baudrate}")
        self.get_logger().info(f"dry_run    : {self.dry_run}")
        self.get_logger().info(f"timeout_sec: {self.timeout_sec}")

        self.run_once()

    def run_once(self):
        try:
            master = mavutil.mavlink_connection(
                self.port,
                baud=self.baudrate,
                autoreconnect=True,
            )

            self.get_logger().info("Waiting for MAVLink heartbeat...")
            master.wait_heartbeat(timeout=self.timeout_sec)

            self.get_logger().info(
                f"Heartbeat received: system={master.target_system}, "
                f"component={master.target_component}"
            )

            if self.dry_run:
                self.get_logger().warn("Dry-run mode enabled. Disarm command will NOT be sent.")
                return

            self.get_logger().warn("Sending DISARM command to Pixhawk...")

            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,  # param1: 1 = arm, 0 = disarm
                0,  # param2: force disarm code if needed, normally 0
                0,
                0,
                0,
                0,
                0,
            )

            ack = self.wait_command_ack(
                master,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                self.timeout_sec,
            )

            if ack is None:
                self.get_logger().error("No COMMAND_ACK received for disarm command.")
                return

            result = ack.result

            if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                self.get_logger().info("DISARM command accepted.")
            else:
                self.get_logger().error(
                    f"DISARM command rejected. MAV_RESULT={result}"
                )

        except Exception as exc:
            self.get_logger().error(f"Disarm node failed: {exc}")

    def wait_command_ack(self, master, command_id: int, timeout_sec: float):
        start_time = time.time()

        while time.time() - start_time < timeout_sec:
            msg = master.recv_match(
                type="COMMAND_ACK",
                blocking=True,
                timeout=1.0,
            )

            if msg is None:
                continue

            if int(msg.command) == int(command_id):
                return msg

        return None


def main(args=None):
    rclpy.init(args=args)

    node = DisarmNode()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()