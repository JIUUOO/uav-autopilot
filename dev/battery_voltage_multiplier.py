from pymavlink import mavutil
import time

SERIAL_PORT = "/dev/cu.usbmodem1101"
BAUD_RATE = 115200

NEW_BATT_VOLT_MULT = 18.48  # 멀티미터 기준으로 계산한 값

conn = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
conn.wait_heartbeat()
print(f"Connected (System Id: {conn.target_system})")

conn.mav.param_set_send(
    conn.target_system,
    conn.target_component,
    b"BATT_VOLT_MULT",
    float(NEW_BATT_VOLT_MULT),
    mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
)

time.sleep(1)

conn.mav.param_request_read_send(
    conn.target_system,
    conn.target_component,
    b"BATT_VOLT_MULT",
    -1,
)

msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=3)
print(msg)
