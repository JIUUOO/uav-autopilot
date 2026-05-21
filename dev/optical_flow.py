from pymavlink import mavutil
import time

PORT = "/dev/ttyACM0"
BAUD = 115200

conn = mavutil.mavlink_connection(PORT, baud=BAUD)
print("[wait] heartbeat...")
conn.wait_heartbeat()
print(f"[ok] heartbeat sys={conn.target_system}, comp={conn.target_component}")

def request_interval(msg_id, hz):
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        msg_id,
        int(1_000_000 / hz),
        0, 0, 0, 0, 0
    )

request_interval(mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 10)
request_interval(mavutil.mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW, 10)

latest_dist = None
latest_flow = None
last_print = 0.0

def fmt_float(x, ndigits=3):
    if x is None:
        return "None"
    return f"{x:.{ndigits}f}"

def fmt_signed(x, ndigits=3):
    if x is None:
        return "None"
    return f"{x:+.{ndigits}f}"

print("[monitor] MTF-02P optical flow / distance")
print("[tip] Ctrl+C to stop")

while True:
    msg = conn.recv_match(
        type=["DISTANCE_SENSOR", "OPTICAL_FLOW"],
        blocking=True,
        timeout=2
    )

    if msg is None:
        print("[warn] no DISTANCE_SENSOR / OPTICAL_FLOW")
        continue

    msg_type = msg.get_type()

    if msg_type == "DISTANCE_SENSOR":
        latest_dist = msg.to_dict()

    elif msg_type == "OPTICAL_FLOW":
        latest_flow = msg.to_dict()

    now = time.time()
    if now - last_print < 0.5:
        continue

    last_print = now

    dist_cm = latest_dist.get("current_distance") if latest_dist else None

    q = latest_flow.get("quality") if latest_flow else None
    ground = latest_flow.get("ground_distance") if latest_flow else None
    comp_x = latest_flow.get("flow_comp_m_x") if latest_flow else None
    comp_y = latest_flow.get("flow_comp_m_y") if latest_flow else None
    rate_x = latest_flow.get("flow_rate_x") if latest_flow else None
    rate_y = latest_flow.get("flow_rate_y") if latest_flow else None

    print(
        f"dist={dist_cm}cm | "
        f"quality={q} | "
        f"ground={fmt_float(ground)}m | "
        f"comp=({fmt_signed(comp_x)}, {fmt_signed(comp_y)}) | "
        f"rate=({fmt_signed(rate_x)}, {fmt_signed(rate_y)})"
    )
