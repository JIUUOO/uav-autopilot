#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node

from uav_bringup.common.mission_config import declare_guided_takeoff_params
from uav_bringup.common.mission_config import load_guided_takeoff_config
from uav_bringup.common.mission_loops import run_hold_loop
from uav_bringup.common.mission_runtime import MissionRuntime


class GuidedTakeoffLoiterNode(Node):
    def __init__(self):
        super().__init__("guided_takeoff_loiter_node")

        declare_guided_takeoff_params(self)
        self.config = load_guided_takeoff_config(self)

        self.stop_event = threading.Event()
        # stop_event only stops this node's internal loops (NTRIP/hold/wait) => it does not change FC state!
        # The Pixhawk may remain in the last mode/armed state unless a separate mode/disarm command is sent.
        self.runtime = MissionRuntime(node=self, config=self.config, stop_event=self.stop_event)

        self.flight_ops = None
        self.run_once()

    def run_once(self):
        """Mission sequence: GUIDED TAKEOFF -> LOITER (hold)."""

        self.runtime.log_config("=== RTK + Optical Flow + GUIDED Takeoff + LOITER ===")

        if not self.runtime.connect_and_prepare(stream_hz=5.0, include_flow_rad=True):
            self.runtime.finish()
            return

        self.flight_ops = self.runtime.make_flight_ops()

        if self.runtime.stop_if_dry_run("Dry-run enabled. GUIDED/ARM/TAKEOFF/LOITER not sent."):
            return

        if not self.flight_ops.set_mode("GUIDED"):
            self.runtime.finish()
            return
        if not self.flight_ops.arm():
            self.runtime.finish()
            return
        if not self.flight_ops.wait_armed():
            self.runtime.finish()
            return
        if not self.flight_ops.takeoff(self.config.altitude_m):
            self.runtime.finish()
            return
        if not self.flight_ops.wait_altitude_reached(self.config.altitude_m, self.config.altitude_ratio):
            self.runtime.finish()
            return
        if not self.flight_ops.set_mode("LOITER"):
            self.runtime.finish()
            return

        self.get_logger().warn("LOITER mode set. Keeping NTRIP forwarding alive.")
        run_hold_loop(
            stop_event=self.stop_event,
            drain_fn=self.runtime.drain_mavlink,
            readiness=self.runtime.readiness,
            logger=self.get_logger(),
            loiter_hold_sec=self.config.loiter_hold_sec,
            ntrip_connected_fn=self.runtime.is_ntrip_connected,
            rtcm_frames_fn=self.runtime.get_rtcm_frames,
        )
        self.runtime.finish()


def main(args=None):
    rclpy.init(args=args)
    node = GuidedTakeoffLoiterNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
