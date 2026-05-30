#!/usr/bin/env python3

import base64
import os
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from pymavlink import mavutil

FIX_NAME = {
    0: "NO_GPS",
    1: "NO_FIX",
    2: "2D_FIX",
    3: "3D_FIX",
    4: "DGPS",
    5: "RTK_FLOAT",
    6: "RTK_FIXED",
}


class RTCMParser:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf.extend(data)
        frames = []

        while True:
            idx = self.buf.find(b"\xd3")
            if idx < 0:
                self.buf.clear()
                break

            if idx > 0:
                del self.buf[:idx]

            if len(self.buf) < 3:
                break

            length = ((self.buf[1] & 0x03) << 8) | self.buf[2]
            total_len = 3 + length + 3

            if length > 1023:
                del self.buf[0]
                continue

            if len(self.buf) < total_len:
                break

            frame = bytes(self.buf[:total_len])
            del self.buf[:total_len]
            frames.append(frame)

        return frames


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

        # initialize
        self.master = None
        self.mav_send_lock = threading.Lock()
        self.stop_event = threading.Event()

        self.gps_fix_type = 0
        self.gps_sats = 0  # number of satellites visible (not used in logic)
        self.gps_hacc_mm = None  # horizontal accuracy
        self.gps_vacc_mm = None  # vertical accuracy
        self.last_gps_time = 0.0

        self.last_flow_time = 0.0
        self.last_rangefinder_time = 0.0

        self.rtcm_bytes = 0
        self.rtcm_frames = 0
        self.ntrip_connected = False

        self.run_once()

    def run_once(self):
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

        self.connect_mavlink()
        self.request_streams()

        if self.enable_ntrip:
            if not self.ntrip_user:
                self.get_logger().error("NTRIP user empty. Set NTRIP_USER or -p ntrip_user:=...")
                return

            threading.Thread(target=self.ntrip_loop, daemon=True).start()

        if not self.wait_ready():
            self.get_logger().error("Readiness check failed.")
            self.stop_event.set()
            return

        if self.dry_run:
            self.get_logger().warn("Dry-run enabled. GUIDED/ARM/TAKEOFF/LOITER not sent.")
            self.stop_event.set()
            return

        if not self.set_mode("GUIDED"):
            self.stop_event.set()
            return

        if not self.arm():
            self.stop_event.set()
            return

        if not self.wait_armed():
            self.stop_event.set()
            return

        if not self.takeoff():
            self.stop_event.set()
            return

        if not self.wait_altitude_reached():
            self.stop_event.set()
            return

        if not self.set_mode("LOITER"):
            self.stop_event.set()
            return

        self.get_logger().warn("LOITER mode set. Keeping NTRIP forwarding alive.")
        self.hold_loop()
        self.stop_event.set()

    def connect_mavlink(self):
        self.master = mavutil.mavlink_connection(
            self.port,
            baud=self.baudrate,
            autoreconnect=True,
        )

        self.get_logger().info("Waiting heartbeat...")
        self.master.wait_heartbeat(timeout=self.timeout_sec)
        self.get_logger().info(f"Heartbeat OK: system={self.master.target_system}, component={self.master.target_component}")

    def request_streams(self):
        msg_ids = [
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            mavutil.mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW,
            mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
            mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
        ]

        if hasattr(mavutil.mavlink, "MAVLINK_MSG_ID_OPTICAL_FLOW_RAD"):
            msg_ids.append(mavutil.mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW_RAD)

        for msg_id in msg_ids:
            self.request_message_interval(msg_id, 5.0)

    def request_message_interval(self, msg_id: int, hz: float):
        interval_us = int(1_000_000 / hz)

        with self.mav_send_lock:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
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

    def connect_ntrip(self):
        sock = socket.create_connection((self.ntrip_host, self.ntrip_port), timeout=10)
        auth = base64.b64encode(f"{self.ntrip_user}:{self.ntrip_pass}".encode()).decode()

        req = f"GET /{self.ntrip_mountpoint} HTTP/1.1\r\n" f"Host: {self.ntrip_host}\r\n" f"User-Agent: ROS2-UAV-NTRIP/0.1\r\n" f"Ntrip-Version: Ntrip/2.0\r\n" f"Authorization: Basic {auth}\r\n" f"Connection: keep-alive\r\n" f"\r\n"

        sock.sendall(req.encode())

        header = b""
        while b"\r\n\r\n" not in header:
            b = sock.recv(1)
            if not b:
                raise RuntimeError("NTRIP closed while reading header")
            header += b

        header_text = header.decode(errors="ignore")
        self.get_logger().info("NTRIP response header:\n" + header_text)

        if "200 OK" not in header_text and "ICY 200 OK" not in header_text:
            raise RuntimeError("NTRIP connection failed")

        is_chunked = "Transfer-Encoding: chunked" in header_text
        return sock, is_chunked

    def read_line(self, sock):
        line = b""
        while not line.endswith(b"\r\n"):
            b = sock.recv(1)
            if not b:
                raise RuntimeError("socket closed")
            line += b
        return line

    def read_exact(self, sock, n: int):
        data = b""
        while len(data) < n:
            chunk = sock.recv(n - len(data))
            if not chunk:
                raise RuntimeError("socket closed")
            data += chunk
        return data

    def ntrip_bytes(self, sock, is_chunked):
        if not is_chunked:
            while not self.stop_event.is_set():
                data = sock.recv(4096)
                if not data:
                    raise RuntimeError("NTRIP stream closed")
                yield data
            return

        while not self.stop_event.is_set():
            line = self.read_line(sock).strip()
            if not line:
                continue

            size = int(line.split(b";")[0], 16)
            if size == 0:
                raise RuntimeError("NTRIP chunked stream ended")

            data = self.read_exact(sock, size)
            self.read_exact(sock, 2)
            yield data

    def send_rtcm_frame(self, frame: bytes):
        max_len = 180

        if len(frame) <= max_len:
            data = list(frame) + [0] * (max_len - len(frame))
            with self.mav_send_lock:
                self.master.mav.gps_rtcm_data_send(0, len(frame), data)
            return

        seq_id = self.rtcm_frames & 0x1F

        for frag_id, i in enumerate(range(0, len(frame), max_len)):
            chunk = frame[i: i + max_len]
            flags = 1 | (frag_id << 1) | (seq_id << 3)
            data = list(chunk) + [0] * (max_len - len(chunk))

            with self.mav_send_lock:
                self.master.mav.gps_rtcm_data_send(flags, len(chunk), data)

    def ntrip_loop(self):
        parser = RTCMParser()

        while not self.stop_event.is_set():
            try:
                sock, is_chunked = self.connect_ntrip()
                self.ntrip_connected = True
                self.get_logger().warn(f"NTRIP connected. chunked={is_chunked}")

                for data in self.ntrip_bytes(sock, is_chunked):
                    self.rtcm_bytes += len(data)

                    for frame in parser.feed(data):
                        self.rtcm_frames += 1
                        self.send_rtcm_frame(frame)

            except Exception as exc:
                self.ntrip_connected = False
                self.get_logger().error(f"NTRIP loop error: {exc}")
                time.sleep(3.0)

    def drain_mavlink(self):
        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break

            msg_type = msg.get_type()
            now = time.time()

            if msg_type == "GPS_RAW_INT":
                self.gps_fix_type = int(msg.fix_type)
                self.gps_sats = int(msg.satellites_visible)
                self.gps_hacc_mm = getattr(msg, "h_acc", None)
                self.gps_vacc_mm = getattr(msg, "v_acc", None)
                self.last_gps_time = now

            elif msg_type in ["OPTICAL_FLOW", "OPTICAL_FLOW_RAD"]:
                self.last_flow_time = now

            elif msg_type == "DISTANCE_SENSOR":
                self.last_rangefinder_time = now

    def gps_ready(self):
        if time.time() - self.last_gps_time > 3.0:
            return False

        if self.gps_fix_type < self.min_gps_fix_type:
            return False

        if self.gps_hacc_mm is not None:
            hacc_m = float(self.gps_hacc_mm) / 1000.0
            if hacc_m > self.max_hacc_m:
                return False

        return True

    def flow_ready(self):
        if not self.require_optical_flow:
            return True

        return (time.time() - self.last_flow_time) <= self.optical_flow_timeout_sec

    def wait_ready(self):
        self.get_logger().warn("Waiting GPS/DGPS/RTK and Optical Flow readiness...")

        start = time.time()
        last_print = 0.0

        while time.time() - start < self.ready_timeout_sec:
            self.drain_mavlink()

            now = time.time()
            if now - last_print > 2.0:
                fix_name = FIX_NAME.get(self.gps_fix_type, "UNKNOWN")
                hacc_m = None if self.gps_hacc_mm is None else self.gps_hacc_mm / 1000.0
                flow_age = None if self.last_flow_time <= 0 else now - self.last_flow_time

                self.get_logger().info(f"ready_check: " f"ntrip={self.ntrip_connected} " f"rtcm_frames={self.rtcm_frames} " f"fix={self.gps_fix_type}({fix_name}) " f"sats={self.gps_sats} " f"hacc_m={hacc_m} " f"flow_age={flow_age}")
                last_print = now

            if self.gps_ready() and self.flow_ready():
                self.get_logger().warn("Preflight readiness OK.")
                return True

            time.sleep(0.05)

        return False

    def set_mode(self, mode_name: str):
        mapping = self.master.mode_mapping()

        if mapping is None or mode_name not in mapping:
            self.get_logger().error(f"Unsupported mode: {mode_name}")
            self.get_logger().error(f"Available modes: {mapping}")
            return False

        mode_id = mapping[mode_name]

        self.get_logger().warn(f"Setting mode: {mode_name}")

        with self.mav_send_lock:
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )

        start = time.time()

        while time.time() - start < self.timeout_sec:
            self.drain_mavlink()
            msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)

            if msg is None:
                continue

            current_mode = mavutil.mode_string_v10(msg)
            self.get_logger().info(f"Current mode: {current_mode}")

            if current_mode.upper() == mode_name.upper():
                return True

        return False

    def arm(self):
        self.get_logger().warn("Sending ARM command...")

        with self.mav_send_lock:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,
                0,
                0,
                0,
                0,
                0,
                0,
            )

        ack = self.wait_command_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)

        if ack is None:
            self.get_logger().error("No ARM ACK.")
            return False

        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self.get_logger().warn("ARM accepted.")
            return True

        self.get_logger().error(f"ARM rejected: result={ack.result}")
        return False

    def wait_armed(self):
        start = time.time()

        while time.time() - start < self.timeout_sec:
            self.drain_mavlink()
            msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)

            if msg is None:
                continue

            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.get_logger().info(f"Armed state: {armed}")

            if armed:
                return True

        return False

    def takeoff(self):
        self.get_logger().warn(f"Sending TAKEOFF: {self.altitude_m:.2f} m")

        with self.mav_send_lock:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                self.altitude_m,
            )

        ack = self.wait_command_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

        if ack is None:
            self.get_logger().warn("No TAKEOFF ACK. Continue altitude monitoring.")
            return True

        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self.get_logger().warn("TAKEOFF accepted.")
            return True

        self.get_logger().error(f"TAKEOFF rejected: result={ack.result}")
        return False

    def wait_altitude_reached(self):
        target_alt_m = self.altitude_m * self.altitude_ratio
        start = time.time()

        self.get_logger().warn(f"Waiting relative_alt >= {target_alt_m:.2f} m")

        while time.time() - start < max(self.timeout_sec, 30.0):
            self.drain_mavlink()

            msg = self.master.recv_match(
                type="GLOBAL_POSITION_INT",
                blocking=True,
                timeout=1.0,
            )

            if msg is None:
                continue

            relative_alt_m = float(msg.relative_alt) / 1000.0
            self.get_logger().info(f"relative_alt={relative_alt_m:.2f} m / target={self.altitude_m:.2f} m")

            if relative_alt_m >= target_alt_m:
                self.get_logger().warn("Altitude threshold reached.")
                return True

        return False

    def wait_command_ack(self, command_id):
        start = time.time()

        while time.time() - start < self.timeout_sec:
            self.drain_mavlink()
            msg = self.master.recv_match(type="COMMAND_ACK", blocking=True, timeout=1.0)

            if msg is None:
                continue

            if int(msg.command) == int(command_id):
                return msg

        return None

    def hold_loop(self):
        start = time.time()
        last_print = 0.0

        while rclpy.ok() and not self.stop_event.is_set():
            self.drain_mavlink()

            now = time.time()
            if now - last_print > 2.0:
                fix_name = FIX_NAME.get(self.gps_fix_type, "UNKNOWN")
                hacc_m = None if self.gps_hacc_mm is None else self.gps_hacc_mm / 1000.0
                flow_age = None if self.last_flow_time <= 0 else now - self.last_flow_time

                self.get_logger().info(f"LOITER hold: " f"ntrip={self.ntrip_connected} " f"rtcm_frames={self.rtcm_frames} " f"fix={self.gps_fix_type}({fix_name}) " f"sats={self.gps_sats} " f"hacc_m={hacc_m} " f"flow_age={flow_age}")
                last_print = now

            if self.hold_sec > 0 and now - start >= self.hold_sec:
                self.get_logger().warn("hold_sec completed.")
                break

            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = GuidedTakeoffLoiterNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
