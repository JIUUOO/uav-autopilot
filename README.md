# UAV Visual Target Localization

ROS 2 research project for detecting and localizing visual targets from a UAV.

The main task is to determine which camera frames should be sent to a vision-language model, detect a target such as a person, approach it using image-space feedback, and estimate its position using RTK GNSS.

## Task

```text
Takeoff and LOITER
-> select an informative camera frame
-> detect the requested target with Gemini
-> approach using bbox feedback
-> point the gimbal downward
-> center above the target
-> estimate the target RTK position
-> return and land
```

The main research contribution is the **best-frame selector**. It combines lightweight OpenCV image-quality metrics with UAV IMU telemetry before sending selected frames to Gemini.

## System Overview

- **Pixhawk 6C + MAVROS**: flight control and telemetry
- **RTK GNSS**: UAV and target-position estimation
- **USB gimbal camera**: visual input
- **Walkera G-3DH gimbal**: PWM-based pitch control
- **Frame Quality Selector**: scores blur, brightness, scene change, and yaw stability
- **Gemini Analyzer**: target detection, confidence, and normalized bounding box generation
- **Target Feedback**: converts bbox error into bounded movement recommendations
- **Top-down Localizer**: centers above the target and records its RTK position
- **rosbag**: records camera, telemetry, VLM reports, mission state, and actuator commands

## Repository Layout

```text
.
|-- ros2_ws/src/
|   |-- uav_bringup/       # Flight missions, MAVLink runtime, and experiment launch files
|   |-- uav_camera/        # USB camera configuration and image flipping
|   |-- uav_gimbal/        # Gimbal PWM controller and scan FSM
|   |-- uav_interfaces/    # Custom ROS 2 messages
|   |-- uav_vision/        # Frame selection, Gemini analysis, tracking, and localization
|   `-- usb_cam/           # USB camera ROS package
|-- docker/                # ROS 2 Humble Docker environment
|-- tools/flight_review/   # Rosbag video rendering and experiment review tools
|-- dev/                   # Standalone development and API experiments
`-- docs/                  # Architecture and experiment figures
```

## Quick Start

Create the local environment file:

```bash
cp docker/.env.example docker/.env
```

Set `GEMINI_API_KEY` and optional NTRIP credentials in `docker/.env`, then start the container:

```bash
docker compose --env-file docker/.env -f docker/compose.yml up -d
docker exec -it uav-autopilot bash
```

Inside the container:

```bash
cd /workspace/uav-autopilot/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Run the integrated vision experiment:

```bash
ros2 launch uav_bringup vision_experiment.launch.py \
  experiment_mode:=selector \
  enable_rosbag:=true \
  dry_run:=true
```

Use `dry_run:=true` while validating camera, Gemini, topics, and parameters without commanding flight or gimbal hardware.

## Experiment Modes

- `baseline`: sends recent raw camera frames directly to Gemini.
- `selector`: sends frames selected using OpenCV quality metrics and IMU yaw rate.
- `topdown_target_localization_mission`: detects a target, approaches it, centers above it, estimates its RTK position, and returns.
- `flight_record.launch.py`: runs a selected flight mission with camera and rosbag recording.

## Offline Review

Rosbags can be converted into videos containing flight telemetry, frame-selector scores, Gemini reports, and detected bounding boxes.

```bash
python3 -m tools.flight_review.render_flight <bag_directory> \
  --output flight_review.mp4
```

See `tools/flight_review/README.md` for additional review tools.

## Notes

- The project is intended for controlled research experiments, not production autonomous flight.
- Flight parameters, PWM ranges, camera orientation, safety radius, and abort behavior must be validated before outdoor use.
- API keys, NTRIP credentials, rosbags, generated videos, and experiment artifacts should remain local and must not be committed.
