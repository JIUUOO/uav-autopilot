# Flight Review

Offline tools for rendering a ROS 2 flight bag as a split-view MP4:

- Left panel: gimbal camera footage with telemetry OSD
- Right panel: rolling altitude, battery, current, and flight-mode graphs

## Install With Conda

Create and activate a dedicated environment:

```bash
conda create -n uav-flight-review python=3.11
conda activate uav-flight-review

python -m pip install -r tools/flight_review/requirements.txt
conda install -c conda-forge ffmpeg
```

Using `python -m pip` ensures the dependencies are installed into the active
Conda environment.

Verify the environment:

```bash
which python
python -m pip show rosbags opencv-python matplotlib
ffmpeg -version
```

## Install Without Conda

```bash
python3 -m pip install -r tools/flight_review/requirements.txt
```

Install `ffmpeg` separately and ensure it is available on `PATH`.

## Render

Run from the repository root:

```bash
python3 -m tools.flight_review.render_flight \
  uav-autopilot-records/bags/flight_20260605_110622 \
  --usb-cam-flip-mode rotate_180 \
  --output uav-autopilot-records/exports/flight_20260605_110622_review.mp4
```

Use `--usb-cam-flip-mode rotate_180` for upside-down recordings. The supported
values match the ROS `usb_cam_flip_mode` parameter: `none`, `vertical`,
`horizontal`, and `rotate_180`.

Use `--help` to see topic, output-size, graph-window, and encoder options.

The renderer accepts either a rosbag directory or its `.db3` file.

On macOS, the generated H.264 MP4 can be opened with QuickTime or VLC.
