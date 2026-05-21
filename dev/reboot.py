from pymavlink import mavutil
import time
import glob

DEV_RUN_REBOOT = True  # Pixhawk reboot 실행

# 포트 설정
SERIAL_PORT_MODE = "fixed"
# "fixed": SERIAL_PORT에 적은 포트 사용
# "auto" : /dev/cu.usbmodem*, /dev/tty.usbmodem*, /dev/ttyACM*, /dev/ttyUSB* 중 heartbeat 잡히는 포트 자동 선택

SERIAL_PORT = "/dev/cu.usbmodem1101"
# 1. 먼저 연결 포트 확인하기
#   macos: ls /dev/tty.* 또는 ls /dev/cu.*
#   linux: ls /dev/ttyACM*
# 2. 예시 output
#   /dev/tty.debug-console            /dev/tty.usbmodem1401
#   /dev/cu.usbmodem1101              /dev/cu.usbmodem1103

BAUD_RATE = 115200  # Pixhawk USB 연결 기본값


def find_pixhawk_port():
    ports = []

    ports += glob.glob("/dev/cu.usbmodem*")
    ports += glob.glob("/dev/tty.usbmodem*")
    ports += glob.glob("/dev/ttyACM*")
    ports += glob.glob("/dev/ttyUSB*")

    ports = sorted(set(ports))

    if not ports:
        raise RuntimeError("No candidate serial ports found")

    for port in ports:
        print(f"[try] {port}")

        try:
            test_conn = mavutil.mavlink_connection(port, baud=BAUD_RATE)
            hb = test_conn.wait_heartbeat(timeout=5)

            if hb:
                print(f"[ok] Pixhawk heartbeat on {port}")
                return port

        except Exception as e:
            print(f"[fail] {port}: {e}")

    raise RuntimeError("No MAVLink heartbeat found")


if SERIAL_PORT_MODE == "auto":
    SERIAL_PORT = find_pixhawk_port()
elif SERIAL_PORT_MODE == "fixed":
    pass
else:
    raise ValueError('SERIAL_PORT_MODE must be "fixed" or "auto"')


conn = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
conn.wait_heartbeat()
print(f"Connected via {SERIAL_PORT}")
print(f"Connected (System Id: {conn.target_system})")  # 1이면 Pixhawk 정상 인식

if DEV_RUN_REBOOT:
    # Pixhawk Reboot
    conn.mav.command_long_send(
        conn.target_system,  # 대상 System ID (Pixhawk=1)
        conn.target_component,  # 대상 Component ID
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,  # confirmation
        1,  # param1: 1=autopilot reboot
        0,  # param2: companion computer action. 0=do nothing
        0,
        0,
        0,
        0,
        0,
    )

    print("Pixhawk reboot command sent")
    time.sleep(1)
