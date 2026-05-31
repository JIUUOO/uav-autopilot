#!/usr/bin/env python3

import os
import threading
import time

import rclpy
from rclpy.node import Node

from uav_bringup.common.flight_ops import FlightOps
from uav_bringup.common.mavlink_client import MavlinkClient
from uav_bringup.common.ntrip import NtripForwarder
from uav_bringup.common.readiness import ReadinessMonitor


class GuidedTakeoffLoiterNode(Node):
    def __init__(self):
        super().__init__("guided_takeoff_loiter_node")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("dry_run", True)

        self.declare_parameter("altitude_m", 1.0)  # objective altitude (GUIDED mode)
        self.declare_parameter("altitude_ratio", 0.85)  # altitude threshold for GUIDED -> LOITER mode
        self.declare_parameter("timeout_sec", 20.0)  # common timeout for heartbeat, ACK, mode, arm waits
        self.declare_parameter("ready_timeout_sec", 90.0)  # max wait for '(GPS, RTK) + optical flow' readiness before takeoff
        self.declare_parameter("hold_sec", 0.0)  # LOITER hold seconds after takeoff; 0.0 means indefinite

        # core of RTK F9P GNSS
        self.declare_parameter("enable_ntrip", True)  # for fix_type to be 6
        self.declare_parameter("ntrip_host", "www.gnssdata.or.kr")  # GNSS provider in Korea
        self.declare_parameter("ntrip_port", 2101)
        self.declare_parameter("ntrip_mountpoint", "SUWN-RTCM31")  # Suwon
        self.declare_parameter("ntrip_user", os.getenv("NTRIP_USER", ""))
        self.declare_parameter("ntrip_pass", os.getenv("NTRIP_PASS", "gnss"))

        self.declare_parameter("min_gps_fix_type", 4)
        self.declare_parameter("max_hacc_m", 5.0)  # ready check fails when hacc_m > max_hacc_m (hacc: horizontal accuracy)

        self.declare_parameter("require_optical_flow", True)
        self.declare_parameter("optical_flow_timeout_sec", 3.0)  # optical flow is ready only if a recent flow message arrived within this timeout

        self.port = str(self.get_parameter("port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)

        self.altitude_m = float(self.get_parameter("altitude_m").value)
        self.altitude_ratio = float(self.get_parameter("altitude_ratio").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.ready_timeout_sec = float(self.get_parameter("ready_timeout_sec").value)
        self.hold_sec = float(self.get_parameter("hold_sec").value)

        self.enable_ntrip = bool(self.get_parameter("enable_ntrip").value)
        self.ntrip_host = str(self.get_parameter("ntrip_host").value)
        self.ntrip_port = int(self.get_parameter("ntrip_port").value)
        self.ntrip_mountpoint = str(self.get_parameter("ntrip_mountpoint").value)
        self.ntrip_user = str(self.get_parameter("ntrip_user").value)
        self.ntrip_pass = str(self.get_parameter("ntrip_pass").value)

        self.min_gps_fix_type = int(self.get_parameter("min_gps_fix_type").value)
        self.max_hacc_m = float(self.get_parameter("max_hacc_m").value)

        self.require_optical_flow = bool(self.get_parameter("require_optical_flow").value)
        self.optical_flow_timeout_sec = float(self.get_parameter("optical_flow_timeout_sec").value)

        self.stop_event = threading.Event()
        # stop_event only stops this node's internal loops (NTRIP/hold/wait) => it does not change FC state!
        # The Pixhawk may remain in the last mode/armed state unless a separate mode/disarm command is sent.

        self.mav_client = MavlinkClient(
            port=self.port,
            baudrate=self.baudrate,
            timeout_sec=self.timeout_sec,
            logger=self.get_logger(),
        )
        self.readiness = ReadinessMonitor(
            logger=self.get_logger(),
            min_gps_fix_type=self.min_gps_fix_type,
            max_hacc_m=self.max_hacc_m,
            require_optical_flow=self.require_optical_flow,
            optical_flow_timeout_sec=self.optical_flow_timeout_sec,
            ready_timeout_sec=self.ready_timeout_sec,
        )
        self.flight_ops = None
        self.ntrip_forwarder = None

        self.run_once()

    def run_once(self):
        """Mission Sequence"""

        self.get_logger().warn("=== RTK + Optical Flow + GUIDED Takeoff + LOITER ===")
        self.get_logger().info(f"port                 : {self.port}")
        self.get_logger().info(f"baudrate             : {self.baudrate}")
        self.get_logger().info(f"dry_run              : {self.dry_run}")
        self.get_logger().info(f"altitude_m           : {self.altitude_m}")
        self.get_logger().info(f"enable_ntrip         : {self.enable_ntrip}")
        self.get_logger().info(f"ntrip_host           : {self.ntrip_host}")
        self.get_logger().info(f"ntrip_mountpoint     : {self.ntrip_mountpoint}")
        self.get_logger().info(f"min_gps_fix_type     : {self.min_gps_fix_type}")
        self.get_logger().info(f"max_hacc_m           : {self.max_hacc_m}")
        self.get_logger().info(f"require_optical_flow : {self.require_optical_flow}")
        self.get_logger().info(f"hold_sec             : {self.hold_sec}")

        self.mav_client.connect()
        self.mav_client.request_default_streams(hz=5.0, include_flow_rad=True)


        # NTRIP
        if self.enable_ntrip:
            if not self.ntrip_user:
                self.get_logger().error("NTRIP user empty. Set NTRIP_USER or -p ntrip_user:=...")
                return

            self.ntrip_forwarder = NtripForwarder(
                mav_client=self.mav_client,
                logger=self.get_logger(),
                stop_event=self.stop_event,
                host=self.ntrip_host,
                port=self.ntrip_port,
                mountpoint=self.ntrip_mountpoint,
                user=self.ntrip_user,
                password=self.ntrip_pass,
            )
            self.ntrip_forwarder.start()

        if not self.readiness.wait_ready(
            drain_fn=self.drain_mavlink,
            stop_event=self.stop_event,
            ntrip_connected_fn=lambda: (
                self.ntrip_forwarder.connected if self.ntrip_forwarder is not None else False
            ),
            rtcm_frames_fn=lambda: (
                self.ntrip_forwarder.frame_count if self.ntrip_forwarder is not None else 0
            ),
        ):
            self.get_logger().error("Readiness check failed.")
            self.stop_event.set()
            return

        self.flight_ops = FlightOps(
            mav_client=self.mav_client,
            logger=self.get_logger(),
            timeout_sec=self.timeout_sec,
            drain_fn=self.drain_mavlink,
        )

        if self.dry_run:
            self.get_logger().warn("Dry-run enabled. GUIDED/ARM/TAKEOFF/LOITER not sent.")
            self.stop_event.set()
            return

        if not self.flight_ops.set_mode("GUIDED"):
            self.stop_event.set()
            return
        if not self.flight_ops.arm():
            self.stop_event.set()
            return
        if not self.flight_ops.wait_armed():
            self.stop_event.set()
            return
        if not self.flight_ops.takeoff(self.altitude_m):
            self.stop_event.set()
            return
        if not self.flight_ops.wait_altitude_reached(self.altitude_m, self.altitude_ratio):
            self.stop_event.set()
            return
        if not self.flight_ops.set_mode("LOITER"):
            self.stop_event.set()
            return

        self.get_logger().warn("LOITER mode set. Keeping NTRIP forwarding alive.")
        self.hold_loop()
        self.stop_event.set()

    def drain_mavlink(self):
        self.mav_client.drain_messages(self.readiness.process_message)

    def hold_loop(self) -> bool:
        start = time.time()
        last_print = 0.0

        while rclpy.ok() and not self.stop_event.is_set():
            self.drain_mavlink()

            now = time.time()
            if now - last_print > 2.0:
                self.readiness.format_status(
                    prefix="LOITER hold",
                    ntrip_connected=(
                        self.ntrip_forwarder.connected if self.ntrip_forwarder is not None else False
                    ),
                    rtcm_frames=(
                        self.ntrip_forwarder.frame_count if self.ntrip_forwarder is not None else 0
                    ),
                    now=now,
                )
                last_print = now

            if self.hold_sec > 0.0 and now - start >= self.hold_sec:
                self.get_logger().warn("hold_sec completed.")
                return True

            time.sleep(0.05)

        return False


def main(args=None):
    rclpy.init(args=args)
    node = GuidedTakeoffLoiterNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
