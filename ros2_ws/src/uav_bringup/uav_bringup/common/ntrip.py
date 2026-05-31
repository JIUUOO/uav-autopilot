#!/usr/bin/env python3

import base64
import socket
import threading
import time


class RTCMParser:
    """Splits incoming RTCM byte stream into complete RTCM frames."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        """
        Binary: [0xD3][len_hi(2bit)|reserved(6bit)][len_lo(8bit)][payload(len bytes)][CRC24Q(3 bytes)]
        Frame: header(3 bytes) + payload(len) + crc(3 bytes)
        """

        self.buf.extend(data)
        frames = []

        while True:
            idx = self.buf.find(b"\xD3")
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


class NtripForwarder:
    """Reads RTCM from NTRIP caster and forwards to FC through MAVLink."""

    def __init__(
        self,
        mav_client,
        logger,
        stop_event,
        host: str,
        port: int,
        mountpoint: str,
        user: str,
        password: str,
    ):
        self.mav_client = mav_client
        self.logger = logger
        self.stop_event = stop_event
        self.host = host
        self.port = port
        self.mountpoint = mountpoint
        self.user = user
        self.password = password

        self.rtcm_bytes = 0
        self.rtcm_frames = 0
        self.ntrip_connected = False
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self.ntrip_loop, daemon=True)
        self._thread.start()

    def connect_ntrip(self):
        sock = socket.create_connection((self.host, self.port), timeout=10)
        auth = base64.b64encode(f"{self.user}:{self.password}".encode()).decode()

        req = (
            f"GET /{self.mountpoint} HTTP/1.1\r\n"
            f"Host: {self.host}\r\n"
            f"User-Agent: ROS2-UAV-NTRIP/0.1\r\n"
            f"Ntrip-Version: Ntrip/2.0\r\n"
            f"Authorization: Basic {auth}\r\n"
            f"Connection: keep-alive\r\n"
            f"\r\n"
        )
        sock.sendall(req.encode())

        header = b""
        while b"\r\n\r\n" not in header:
            b = sock.recv(1)
            if not b:
                raise RuntimeError("NTRIP closed while reading header")
            header += b

        header_text = header.decode(errors="ignore")
        self.logger.info("NTRIP response header:\n" + header_text)

        if "200 OK" not in header_text and "ICY 200 OK" not in header_text:
            raise RuntimeError("NTRIP connection failed")

        is_chunked = "Transfer-Encoding: chunked" in header_text
        return sock, is_chunked

    @staticmethod
    def _read_line(sock):
        line = b""
        while not line.endswith(b"\r\n"):
            b = sock.recv(1)
            if not b:
                raise RuntimeError("socket closed")
            line += b
        return line

    @staticmethod
    def _read_exact(sock, n: int):
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
            line = self._read_line(sock).strip()
            if not line:
                continue

            size = int(line.split(b";")[0], 16)
            if size == 0:
                raise RuntimeError("NTRIP chunked stream ended")

            data = self._read_exact(sock, size)
            self._read_exact(sock, 2)
            yield data

    def send_rtcm_frame(self, frame: bytes):
        max_len = 180

        if len(frame) <= max_len:
            data = list(frame) + [0] * (max_len - len(frame))
            with self.mav_client.send_lock:
                self.mav_client.master.mav.gps_rtcm_data_send(0, len(frame), data)
            return

        seq_id = self.rtcm_frames & 0x1F

        for frag_id, i in enumerate(range(0, len(frame), max_len)):
            chunk = frame[i:i + max_len]
            flags = 1 | (frag_id << 1) | (seq_id << 3)
            data = list(chunk) + [0] * (max_len - len(chunk))
            with self.mav_client.send_lock:
                self.mav_client.master.mav.gps_rtcm_data_send(flags, len(chunk), data)

    def ntrip_loop(self):
        parser = RTCMParser()

        while not self.stop_event.is_set():
            sock = None
            try:
                sock, is_chunked = self.connect_ntrip()
                self.ntrip_connected = True
                self.logger.warn(f"NTRIP connected. chunked={is_chunked}")

                for data in self.ntrip_bytes(sock, is_chunked):
                    self.rtcm_bytes += len(data)
                    for frame in parser.feed(data):
                        self.rtcm_frames += 1
                        self.send_rtcm_frame(frame)
            except Exception as exc:
                self.ntrip_connected = False
                self.logger.error(f"NTRIP loop error: {exc}")
                time.sleep(3.0)
            finally:
                if sock is not None:
                    try:
                        sock.close()
                    except Exception:
                        pass
