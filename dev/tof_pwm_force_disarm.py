from pymavlink import mavutil
import time

# Motor PWM throttle up 후 tof 50cm 이상이 되면 Motor Stop

# 환경 설정
DEV_MODE = True  # True: 개발/테스트용, False: 실제 비행
DEV_DISABLE_ARMING_CHECK = True  # 안전 체크 비활성화 (지상 테스트용)
DEV_RUN_ARM_DISARM = True  # Arm/Disarm 시퀀스 실행

SERIAL_PORT = "/dev/tty.usbmodem1101"
# 1. 먼저 연결 포트 확인하기
#   macos: ls /dev/tty.*
#   linux: ls /dev/ttyACM*
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


# ToF 설정
TOF_THRESHOLD_CM = 50  # 50cm 이상이면 모터 출력 차단
TOF_TRIGGER_COUNT = 3  # 연속 3번 이상 넘으면 trigger
TOF_TIMEOUT_SEC = 30  # 최대 테스트 시간

# RC Override 설정
# ArduCopter 기본 throttle channel = RC3
RC_ROLL_PWM = 1500
RC_PITCH_PWM = 1500
RC_THROTTLE_PWM = 1200  # 모터 출력 상수. 처음엔 낮게. 필요하면 1250, 1300 순서로 올림
RC_YAW_PWM = 1500
RC_STOP_PWM = 1000  # throttle low
RC_SEND_HZ = 20  # 초당 20번 override 송신


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


def send_rc_override(conn, throttle_pwm):
    # RC_CHANNELS_OVERRIDE
    # ch1 roll, ch2 pitch, ch3 throttle, ch4 yaw
    conn.mav.rc_channels_override_send(
        conn.target_system,
        conn.target_component,
        RC_ROLL_PWM,
        RC_PITCH_PWM,
        throttle_pwm,
        RC_YAW_PWM,
        0,
        0,
        0,
        0,
    )


def clear_rc_override(conn):
    # 0을 보내면 override 해제
    conn.mav.rc_channels_override_send(
        conn.target_system,
        conn.target_component,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def force_disarm(conn):
    print("FORCE DISARM triggered")

    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,  # param1: 0 = disarm
        21196,  # param2: force disarm
        0,
        0,
        0,
        0,
        0,
    )


def wait_for_distance_sensor(conn, timeout_sec=5):
    print("Checking DISTANCE_SENSOR...")

    start = time.time()

    while time.time() - start < timeout_sec:
        msg = conn.recv_match(type="DISTANCE_SENSOR", blocking=True, timeout=1)

        if msg is None:
            continue

        data = msg.to_dict()
        dist_cm = data.get("current_distance")

        if dist_cm is not None and dist_cm > 0:
            print(f"DISTANCE_SENSOR OK: {dist_cm}cm")
            return True

    print("DISTANCE_SENSOR not detected or invalid")
    return False


conn = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
conn.wait_heartbeat()
print(f"Connected (System Id: {conn.target_system})")  # 1이면 Pixhawk 정상 인식

if DEV_MODE and DEV_DISABLE_ARMING_CHECK:
    conn.mav.param_set_send(
        conn.target_system,  # 대상 System ID (Pixhawk=1)
        conn.target_component,  # 대상 Component ID
        b"ARMING_CHECK",  # Parameter 이름
        0,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32,
    )
    time.sleep(1)
    print("ARMING_CHECK disabled")

# ToF 메시지 요청
request_message_interval(
    conn,
    mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
    20,
)

time.sleep(1)

# ToF 센서 확인
if not wait_for_distance_sensor(conn, timeout_sec=5):
    print("ERROR: ToF sensor not ready. Abort.")
    raise SystemExit(1)

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
    print("Armed")
    time.sleep(2)

    print(f"Motor PWM override started: throttle={RC_THROTTLE_PWM}")
    print(f"ToF threshold: {TOF_THRESHOLD_CM}cm")

    over_count = 0
    start = time.time()
    last_rc_send = 0
    last_print = 0

    try:
        while time.time() - start < TOF_TIMEOUT_SEC:
            now = time.time()

            # throttle PWM은 계속 반복 송신해야 유지됨
            if now - last_rc_send >= 1.0 / RC_SEND_HZ:
                last_rc_send = now
                send_rc_override(conn, RC_THROTTLE_PWM)

            msg = conn.recv_match(type="DISTANCE_SENSOR", blocking=True, timeout=0.02)

            if msg is None:
                continue

            data = msg.to_dict()
            dist_cm = data.get("current_distance")
            orientation = data.get("orientation")
            signal_quality = data.get("signal_quality")

            if dist_cm is None:
                continue

            if dist_cm >= TOF_THRESHOLD_CM:
                over_count += 1
                state = "OVER"
            else:
                over_count = 0
                state = "OK"

            if now - last_print >= 0.2:
                last_print = now
                print(
                    f"dist={dist_cm:4d}cm | "
                    f"orientation={orientation} | "
                    f"signal_quality={signal_quality} | "
                    f"throttle_pwm={RC_THROTTLE_PWM} | "
                    f"{state} {over_count}/{TOF_TRIGGER_COUNT}"
                )

            if over_count >= TOF_TRIGGER_COUNT:
                print(f"ToF threshold reached: {dist_cm}cm >= {TOF_THRESHOLD_CM}cm")
                force_disarm(conn)
                break

        else:
            print("Timeout reached")
            force_disarm(conn)

    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        force_disarm(conn)
