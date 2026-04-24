from pymavlink import mavutil

# 1. 먼저 연결 포트 확인하기
#   ls /dev/tty.*
# 2. 예시 output
#   /dev/tty.debug-console            /dev/tty.usbmodem1401

SERIAL_PORT = "/dev/tty.usbmodem1401"
BAUD_RATE = 115200  # Pixhawk USB 연결 기본값

conn = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
conn.wait_heartbeat()
print(f"Connected! System ID: {conn.target_system}")  # 1이면 Pixhawk 정상 인식
