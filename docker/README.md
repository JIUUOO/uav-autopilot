# UAV Autopilot Docker

This image is for ROS 2 Humble development and Linux companion-computer execution.

## Build

From the repository root:

```bash
docker compose -f docker/compose.yml build
```

## Start A Shell

```bash
docker compose -f docker/compose.yml run --rm uav-autopilot bash
```

Inside the container:

```bash
cd /workspace/uav-autopilot/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run The Vision Experiment

Set secrets on the host before starting the container:

```bash
export GEMINI_API_KEY="..."
export NTRIP_USER="nninjiuuoo@gmail.com"
export NTRIP_PASS="gnss"
```

Or use an env file:

```bash
cp docker/.env docker/.env
# edit docker/.env
docker compose --env-file docker/.env.example -f docker/compose.yml run --rm uav-autopilot bash
```

Then run:

```bash
docker compose -f docker/compose.yml run --rm uav-autopilot bash
```

Inside the container:

```bash
cd /workspace/uav-autopilot/ros2_ws
source install/setup.bash

ros2 launch uav_bringup vision_experiment.launch.py \
  experiment_mode:=selector \
  enable_usb_cam:=true \
  enable_selector:=true \
  enable_gemini:=true \
  enable_mission_node:=true \
  mission_executable:=bounded_scout_mission \
  dry_run:=false \
  mission_altitude_m:=3.5 \
  scout_radius_m:=3.0 \
  corner_offset_m:=3.0 \
  software_radius_m:=6.5 \
  enable_low_altitude_inspection:=false \
  disable_inspection_for_person:=true \
  abort_mode:=LOITER \
  enable_battery_monitor:=false \
  enable_rosbag:=true \
  bag_name:=docker_selector_bounded_scout_01
```

## Hardware Notes

On Linux companion computers, `privileged: true` and `/dev:/dev` allow access to Pixhawk serial devices and USB cameras.

On macOS Docker Desktop, direct USB serial and webcam passthrough is limited. Use Docker mainly for build/development there. For real Pixhawk/camera flight execution, run this container on a Linux host or companion computer.

`battery_monitor_node` opens a MAVLink serial port directly. Do not enable it when MAVROS already owns the same Pixhawk serial port. `/mavros/battery` is still recorded by rosbag.

YDLidar dev scripts require the optional `external/YDLidar-SDK` submodule and are not installed by this base image.
