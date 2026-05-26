#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from pymavlink import mavutil


class ArmNode(Node):
    """
    Pixhawk arm command node
    
    - Connect to Pixhawk through MAVLink
    - Wait for heartbeat
    - Send arm command
    - Wait for COMMAND_ACK
    - Exit
    
    ArduPilot may auto-disarm if throttle stays low
    """

    def __init__(self):
        super().__init__("arm_node")

        # parameter
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200) # pixhawk usb: 115200
        self.declare_parameter("dry_run", True) # True: only check pixhawk heartbeat (no arm command)
        self.declare_parameter("timeout_sec", 10.0) # heartbeat waiting timeout

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)

        self.get_logger().info("Arm node started.")
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
                self.get_logger().warn("Dry-run mode enabled. Arm command will NOT be sent.")
                return

            self.get_logger().warn("Sending ARM command to Pixhawk...")

            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,  # 1 = arm, 0 = disarm
                0,
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
                self.get_logger().error("No COMMAND_ACK received for arm command.")
                return

            result = ack.result

            if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                self.get_logger().info("ARM command accepted.")
            else:
                self.get_logger().error(f"ARM command rejected. MAV_RESULT={result}")

        except Exception as exc:
            self.get_logger().error(f"Arm node failed: {exc}")

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

    node = ArmNode()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()