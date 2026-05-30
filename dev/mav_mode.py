import time
from pymavlink import mavutil

PORT = "/dev/ttyACM0"
BAUD = 115200

m = mavutil.mavlink_connection(PORT, baud=BAUD, autoreconnect=True)

print(f"[MAVLink] connecting {PORT} @ {BAUD}")
m.wait_heartbeat()
print(f"[MAVLink] heartbeat OK system={m.target_system} component={m.target_component}")

def request_msg(msg_id, hz):
    interval_us = int(1_000_000 / hz)
    m.mav.command_long_send(
        m.target_system,
        m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        msg_id,
        interval_us,
        0, 0, 0, 0, 0,
    )

request_msg(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 2)
request_msg(mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 5)
request_msg(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1)

last_mode = None
last_armed = None
last_rc_print = 0

while True:
    msg = m.recv_match(blocking=True, timeout=1.0)
    if msg is None:
        print("[WAIT] no msg")
        continue

    t = msg.get_type()
    now = time.time()

    if t == "HEARTBEAT":
        mode = mavutil.mode_string_v10(msg)
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        if mode != last_mode or armed != last_armed:
            print(f"[MODE] mode={mode} armed={armed} base_mode={msg.base_mode} custom_mode={msg.custom_mode}")
            last_mode = mode
            last_armed = armed

    elif t == "RC_CHANNELS":
        if now - last_rc_print > 0.5:
            ch = [
                msg.chan1_raw,
                msg.chan2_raw,
                msg.chan3_raw,
                msg.chan4_raw,
                msg.chan5_raw,
                msg.chan6_raw,
                msg.chan7_raw,
                msg.chan8_raw,
            ]
            print(f"[RC] rssi={msg.rssi} ch1-8={ch}")
            last_rc_print = now

    elif t == "STATUSTEXT":
        print(f"[TEXT] severity={msg.severity} text={msg.text}")