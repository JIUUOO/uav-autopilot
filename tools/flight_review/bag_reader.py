"""Read camera frames and telemetry from ROS 2 bags."""

from bisect import bisect_left
from pathlib import Path

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore


MAVROS_STATE_DEFINITION = """
std_msgs/Header header
bool connected
bool armed
bool guided
bool manual_input
string mode
uint8 system_status
"""


def find_nearest(times: list[float], target: float) -> int | None:
    """Return the index of the value nearest to target in a sorted list."""
    if not times:
        return None

    index = bisect_left(times, target)
    if index == 0:
        return 0
    if index == len(times):
        return len(times) - 1
    if target - times[index - 1] <= times[index] - target:
        return index - 1
    return index


class TelemetryStore:
    """Telemetry values indexed by rosbag timestamp."""

    def __init__(self):
        self.alt_t: list[float] = []
        self.alt_v: list[float] = []
        self.alt_home: float | None = None

        self.batt_t: list[float] = []
        self.volt_v: list[float] = []
        self.curr_v: list[float] = []
        self.pct_v: list[float] = []

        self.state_t: list[float] = []
        self.modes: list[str] = []
        self.armed: list[bool] = []

    def alt_rel(self, index: int) -> float:
        """Return altitude relative to the first recorded GPS altitude."""
        if self.alt_home is None:
            return 0.0
        return self.alt_v[index] - self.alt_home

    def current_telem(self, timestamp: float) -> dict:
        """Return telemetry values nearest to a timestamp."""
        output = {}

        if self.alt_t:
            index = find_nearest(self.alt_t, timestamp)
            output["alt_abs"] = self.alt_v[index]
            output["alt_rel"] = self.alt_rel(index)
        else:
            output["alt_abs"] = output["alt_rel"] = 0.0

        if self.batt_t:
            index = find_nearest(self.batt_t, timestamp)
            output["voltage"] = self.volt_v[index]
            output["current"] = abs(self.curr_v[index])
            output["pct"] = self.pct_v[index] * 100
        else:
            output["voltage"] = output["current"] = output["pct"] = 0.0

        if self.state_t:
            index = find_nearest(self.state_t, timestamp)
            output["mode"] = self.modes[index]
            output["armed"] = self.armed[index]
        else:
            output["mode"] = "?"
            output["armed"] = False

        return output

    @staticmethod
    def window(times, values, timestamp, duration):
        """Return values inside a rolling window ending at timestamp."""
        start = bisect_left(times, timestamp - duration)
        end = bisect_left(times, timestamp)
        if end < len(times) and times[end] <= timestamp:
            end += 1
        return times[start:end], values[start:end]


def load_bag(
    bag_path: str,
    cam_topic: str,
    alt_topic: str,
    batt_topic: str,
    state_topic: str,
):
    """Read the requested camera and telemetry topics from a ROS 2 bag."""
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    typestore.register(
        get_types_from_msg(MAVROS_STATE_DEFINITION, "mavros_msgs/msg/State")
    )
    frames = []
    store = TelemetryStore()

    path = Path(bag_path)
    bag_dir = str(path.parent if path.is_file() and path.suffix == ".db3" else path)
    print(f"[bag] Opening: {bag_dir}")

    with Reader(bag_dir) as reader:
        topics_in_bag = {connection.topic for connection in reader.connections}
        print(f"[bag] Topics found: {sorted(topics_in_bag)}")

        requested_topics = {cam_topic, alt_topic, batt_topic, state_topic}
        for topic in requested_topics:
            if topic not in topics_in_bag:
                print(f"[warn] Topic not found in bag: {topic}")

        connections = [
            connection
            for connection in reader.connections
            if connection.topic in requested_topics
        ]
        for connection, timestamp_ns, rawdata in reader.messages(
            connections=connections
        ):
            timestamp = timestamp_ns * 1e-9
            topic = connection.topic
            try:
                message = typestore.deserialize_cdr(rawdata, connection.msgtype)
            except Exception as error:
                print(f"[warn] Deserialize error on {topic}: {error}")
                continue

            if topic == cam_topic:
                frames.append(
                    (
                        timestamp,
                        bytes(message.data),
                        getattr(message, "encoding", "jpeg"),
                        getattr(message, "height", 0),
                        getattr(message, "width", 0),
                    )
                )
            elif topic == alt_topic:
                store.alt_t.append(timestamp)
                store.alt_v.append(message.altitude)
                if store.alt_home is None:
                    store.alt_home = message.altitude
            elif topic == batt_topic:
                store.batt_t.append(timestamp)
                store.volt_v.append(message.voltage)
                store.curr_v.append(message.current)
                store.pct_v.append(max(0.0, message.percentage))
            elif topic == state_topic:
                store.state_t.append(timestamp)
                store.modes.append(message.mode if message.mode else "?")
                store.armed.append(bool(message.armed))

    print(f"[bag] Camera frames  : {len(frames)}")
    print(f"[bag] Altitude msgs  : {len(store.alt_t)}")
    print(f"[bag] Battery msgs   : {len(store.batt_t)}")
    print(f"[bag] State msgs     : {len(store.state_t)}")
    return frames, store
