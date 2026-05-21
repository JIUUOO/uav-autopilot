from pymavlink import mavutil

# 환경 설정
SERIAL_PORT = "/dev/cu.usbmodem1101"
# 1. 먼저 연결 포트 확인하기
#   macos: ls /dev/tty.* 또는 ls /dev/cu.*
#   linux: ls /dev/ttyACM*
# 2. 예시 output
#   /dev/cu.usbmodem1101              /dev/cu.usbmodem1103
#   /dev/ttyACM0

BAUD_RATE = 115200  # Pixhawk USB 연결 기본값
PRINT_HZ = 1  # 초당 출력 횟수


def request_message_interval(conn, msg_id, hz):
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        msg_id,
        int(1_000_000 / hz),
        0,
        0,
        0,
        0,
        0,
    )


conn = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
conn.wait_heartbeat()
print(f"Connected via {SERIAL_PORT}")
print(f"Connected (System Id: {conn.target_system})")  # 1이면 Pixhawk 정상 인식

# SYS_STATUS 메시지 요청
request_message_interval(
    conn,
    mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
    PRINT_HZ,
)

print("Battery voltage monitor started")
print("Press Ctrl+C to stop")

try:
    while True:
        msg = conn.recv_match(type="SYS_STATUS", blocking=True, timeout=2)

        if msg is None:
            print("No SYS_STATUS message")
            continue

        data = msg.to_dict()

        voltage_mv = data.get("voltage_battery")

        if voltage_mv is None or voltage_mv <= 0 or voltage_mv >= 65535:
            print("battery_voltage = unknown")
            continue

        voltage_v = voltage_mv / 1000.0  # mV -> V

        print(f"battery_voltage = {voltage_v:.2f} V")

except KeyboardInterrupt:
    print("\nBattery monitor stopped")
