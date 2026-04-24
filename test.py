from pymavlink import mavutil
import time

# Python Script → MAVLink → Pixhawk → PWM → ESC → Motor → Propeller

# 환경 설정
DEV_MODE = True  # True: 개발/테스트용, False: 실제 비행
DEV_DISABLE_ARMING_CHECK = True  # 안전 체크 비활성화 (지상 테스트용)
DEV_RUN_ARM_DISARM = True  # Arm/Disarm 시퀀스 실행

SERIAL_PORT = "/dev/tty.usbmodem1401"
# 1. 먼저 연결 포트 확인하기
#   ls /dev/tty.*
# 2. 예시 output
#   /dev/tty.debug-console            /dev/tty.usbmodem1401

BAUD_RATE = 115200  # Pixhawk USB 연결 기본값

FLIGHT_MODE = "STABILIZE"
# ArduPilot Flight Modes (주요 7가지)
# STABILIZE : gyro/accelerometer로 자세만 유지. GPS 불필요. 스틱을 놓으면 수평 유지하지만 위치는 잡지 않음
# ALTHOLD   : STABILIZE + barometer 기반 고도 유지
# LOITER    : ALTHOLD + GPS 기반 위치 고정. 스틱을 놓으면 제자리 유지
# GUIDED    : companion computer가 MAVLink로 목표 좌표 전달. 자율비행에 사용
# AUTO      : 미리 준비한 waypoint 미션 자동 수행
# LAND      : 자동 착륙 sequence
# RTL       : Return To Launch. 이륙 지점으로 복귀 후 착륙


conn = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
conn.wait_heartbeat()
print(f"Connected! System ID: {conn.target_system}")  # 1이면 Pixhawk 정상 인식

if DEV_MODE and DEV_DISABLE_ARMING_CHECK:
    conn.mav.param_set_send(
        conn.target_system,  # 대상 System ID (Pixhawk=1)
        conn.target_component,  # 대상 Component ID
        b"ARMING_CHECK",  # Parameter 이름
        0,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32,
    )
    time.sleep(1)
    print("[DEV] ARMING_CHECK 비활성화됨")

conn.set_mode(FLIGHT_MODE)
time.sleep(2)

if DEV_MODE and DEV_RUN_ARM_DISARM:
    # Arm
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        1,  # param1: 1=arm, 0=disarm
        0,  # 나머지 파라미터
        0,
        0,
        0,
        0,
        0,
    )
    print("Armed - motors running")
    time.sleep(3)  # ARM 유지 시간

    # Disarm
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,  # param1: 1=arm, 0=disarm
        0,
        0,
        0,
        0,
        0,
        0,
    )
    print("Disarmed")
