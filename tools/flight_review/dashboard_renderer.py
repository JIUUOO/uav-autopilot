"""Render telemetry graphs and an on-screen display for flight footage."""

import cv2
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

try:
    from .bag_reader import TelemetryStore, find_nearest
except ImportError:
    from bag_reader import TelemetryStore, find_nearest

C_BG = "#000000"
C_ALT = "#378ADD"
C_VOLT = "#E24B4A"
C_CURR = "#1D9E75"
C_TEXT = "#e6edf3"
C_GRID = "#21262d"

MODE_COLORS = {
    "GUIDED": "#1D9E75",
    "LOITER": "#E6873A",
    "LAND": "#7F77DD",
    "RTL": "#E24B4A",
    "ALT_HOLD": "#BA7517",
}


class GraphRenderer:
    """Render rolling telemetry graphs into an OpenCV image."""

    def __init__(self, width: int, height: int, graph_sec: float):
        self.width = width
        self.height = height
        self.graph_sec = graph_sec
        dpi = 100
        self.fig = plt.figure(
            figsize=(width / dpi, height / dpi), dpi=dpi, facecolor=C_BG
        )
        grid = GridSpec(
            4,
            1,
            figure=self.fig,
            hspace=0.55,
            top=0.93,
            bottom=0.07,
            left=0.10,
            right=0.97,
        )
        self.ax_alt = self.fig.add_subplot(grid[0])
        self.ax_volt = self.fig.add_subplot(grid[1])
        self.ax_curr = self.fig.add_subplot(grid[2])
        self.ax_mode = self.fig.add_subplot(grid[3])

    @staticmethod
    def _reset_axis(axis):
        axis.cla()
        axis.set_facecolor(C_BG)
        axis.grid(True, color=C_GRID, linewidth=0.5)
        axis.tick_params(colors=C_TEXT, labelsize=7)
        for spine in axis.spines.values():
            spine.set_edgecolor(C_GRID)

    def render(self, store: TelemetryStore, timestamp: float) -> np.ndarray:
        self._render_altitude(store, timestamp)
        self._render_voltage(store, timestamp)
        self._render_current(store, timestamp)
        self._render_mode(store, timestamp)

        self.fig.canvas.draw()
        buffer = np.frombuffer(self.fig.canvas.buffer_rgba(), dtype=np.uint8)
        buffer = buffer.reshape(self.height, self.width, 4)
        return cv2.cvtColor(buffer, cv2.COLOR_RGBA2BGR)

    @staticmethod
    def _extend_to_now(times, values, timestamp):
        """Hold the latest telemetry value to the current video-frame timestamp."""
        relative_times = [value - timestamp for value in times]
        values = list(values)
        if relative_times and relative_times[-1] < 0.0:
            relative_times.append(0.0)
            values.append(values[-1])
        return relative_times, values

    def _render_altitude(self, store, timestamp):
        axis = self.ax_alt
        self._reset_axis(axis)
        if store.alt_t:
            values = [store.alt_rel(i) for i in range(len(store.alt_t))]
            times, values = store.window(
                store.alt_t, values, timestamp, self.graph_sec
            )
            if times:
                relative_times, values = self._extend_to_now(times, values, timestamp)
                axis.plot(relative_times, values, color=C_ALT, linewidth=1.5)
                axis.fill_between(relative_times, values, alpha=0.15, color=C_ALT)
                axis.set_title(
                    f"ALT  {values[-1]:+.2f} m AGL",
                    fontsize=8,
                    color=C_TEXT,
                    pad=2,
                )
        self._finish_axis(axis, "m")
        axis.axhline(0, color=C_GRID, linewidth=0.8, linestyle="--")

    def _render_voltage(self, store, timestamp):
        axis = self.ax_volt
        self._reset_axis(axis)
        if store.batt_t:
            times, values = store.window(
                store.batt_t, store.volt_v, timestamp, self.graph_sec
            )
            if times:
                relative_times, values = self._extend_to_now(times, values, timestamp)
                axis.plot(relative_times, values, color=C_VOLT, linewidth=1.5)
                axis.fill_between(relative_times, values, alpha=0.12, color=C_VOLT)
                percentage = store.pct_v[find_nearest(store.batt_t, timestamp)] * 100
                axis.set_title(
                    f"BATT  {values[-1]:.3f} V  ({percentage:.0f}%)",
                    fontsize=8,
                    color=C_TEXT,
                    pad=2,
                )
        self._finish_axis(axis, "V")

    def _render_current(self, store, timestamp):
        axis = self.ax_curr
        self._reset_axis(axis)
        if store.batt_t:
            currents = [abs(value) for value in store.curr_v]
            times, values = store.window(
                store.batt_t, currents, timestamp, self.graph_sec
            )
            if times:
                relative_times, values = self._extend_to_now(times, values, timestamp)
                axis.plot(relative_times, values, color=C_CURR, linewidth=1.5)
                axis.fill_between(relative_times, values, alpha=0.12, color=C_CURR)
                axis.set_title(
                    f"CURRENT  {values[-1]:.1f} A",
                    fontsize=8,
                    color=C_TEXT,
                    pad=2,
                )
        self._finish_axis(axis, "A")

    def _render_mode(self, store, timestamp):
        axis = self.ax_mode
        self._reset_axis(axis)
        for index, start in enumerate(store.state_t):
            relative_start = start - timestamp
            relative_end = (
                store.state_t[index + 1] - timestamp
                if index + 1 < len(store.state_t)
                else 0.0
            )
            if relative_end < -self.graph_sec or relative_start > 0:
                continue
            relative_start = max(relative_start, -self.graph_sec)
            relative_end = min(relative_end, 0.0)
            mode = store.modes[index]
            axis.barh(
                0,
                relative_end - relative_start,
                left=relative_start,
                height=0.6,
                color=MODE_COLORS.get(mode, "#555577"),
                alpha=0.85 if store.armed[index] else 0.35,
            )
            if relative_end - relative_start > 1.5:
                axis.text(
                    (relative_start + relative_end) / 2,
                    0,
                    mode,
                    ha="center",
                    va="center",
                    fontsize=6.5,
                    color="white",
                    fontweight="bold",
                )
        axis.set_xlim(-self.graph_sec, 0)
        axis.set_ylim(-0.5, 0.5)
        axis.set_yticks([])
        axis.set_title("MODE", fontsize=8, color=C_TEXT, pad=2)
        axis.set_xlabel("seconds ago", fontsize=7, color=C_TEXT)

    def _finish_axis(self, axis, ylabel):
        axis.set_xlim(-self.graph_sec, 0)
        axis.set_ylabel(ylabel, fontsize=7, color=C_TEXT)


def draw_osd(frame: np.ndarray, telemetry: dict, elapsed: float) -> np.ndarray:
    """Overlay current telemetry values on a camera frame."""
    height, width = frame.shape[:2]
    overlay = frame.copy()

    def put(text, position, scale=0.55, thick=1, color=(255, 255, 255)):
        cv2.putText(
            overlay,
            text,
            position,
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            (0, 0, 0),
            thick + 2,
            cv2.LINE_AA,
        )
        cv2.putText(
            overlay,
            text,
            position,
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            thick,
            cv2.LINE_AA,
        )

    mode = telemetry.get("mode", "?")
    armed = telemetry.get("armed", False)
    cv2.rectangle(overlay, (0, 0), (width, 28), (0, 0, 0), -1)
    frame = cv2.addWeighted(overlay, 0.6, frame, 0.4, 0)
    overlay = frame.copy()

    rgb = matplotlib.colors.to_rgb(MODE_COLORS.get(mode, "#aaaaaa"))
    mode_color = tuple(int(value * 255) for value in rgb)
    armed_color = (80, 220, 80) if armed else (80, 80, 220)

    put(mode, (6, 20), scale=0.58, color=mode_color)
    put("ARMED" if armed else "DISARMED", (90, 20), scale=0.50, color=armed_color)
    put(f"ALT {telemetry.get('alt_rel', 0.0):+.2f}m", (width // 2 - 55, 20))
    put(
        f"{telemetry.get('voltage', 0.0):.2f}V  "
        f"{telemetry.get('current', 0.0):.1f}A  "
        f"{telemetry.get('pct', 0.0):.0f}%",
        (width - 185, 20),
        scale=0.50,
    )
    put(
        f"{int(elapsed // 60):02d}:{elapsed % 60:05.2f}",
        (6, height - 8),
        scale=0.48,
        color=(180, 180, 180),
    )
    return cv2.addWeighted(overlay, 0.85, frame, 0.15, 0)
