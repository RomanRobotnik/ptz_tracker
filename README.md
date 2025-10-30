# PTZ Tracker

Action-based pan–tilt–zoom tracker for ROS 2.  
Subscribes to `vision_msgs/Detection2DArray`, commands a `FollowJointTrajectory` PTZ controller, and exposes the `TrackTarget` action (`~/start_tracking`) to start/stop tracking on demand.

## Features
- Action server: start on goal, stop on cancel/target loss, feedback with current pan/tilt/zoom.
- Works with Generic detections (default `/person_detector_node/detection_array`).
- PTZ gains, FOV, joints, timeouts configurable via YAML.
- Launch file that loads parameters and handles `use_sim_time`.

## Installation
```bash
cd ~/workspaces/robotnik_sim_ws
colcon build --packages-select ptz_tracker ptz_tracker_interfaces
source install/setup.bash
```

## Launch
```bash
ros2 launch ptz_tracker ptz_tracker.launch.py \
  config_file:=/path/to/ptz_tracker.yaml```
Default config lives in `config/ptz_tracker.yaml`.

## Send Action Goal
```bash
ros2 action send_goal /ptz_tracker/start_tracking ptz_tracker_interfaces/action/TrackTarget "{start: true}"
```

## Parameters (excerpt)
| Parameter | Default | Description |
|-----------|---------|-------------|
| `detection_topic` | `/person_detector_node/detection_array` | Incoming detections |
| `image_topic` | `/robot/top_ptz_rgbd_camera/color/image_raw` | Camera stream (for width/height) |
| `pan_joint` / `tilt_joint` / `zoom_joint` | `robot_top_ptz_camera_*` | Joint names |
| `kp_pan` / `kp_tilt` / `kp_zoom` | `0.2 / 0.1 / 0.1` | Proportional gains |
| `command_period` | `0.2` s | Control loop period |
| `detection_timeout` | `3.0` s | Time without detections before abort |

Adjust via YAML or CLI (`--ros-args -p key:=value`).

## Dependencies
- `rclpy`, `control_msgs`, `trajectory_msgs`, `sensor_msgs`, `vision_msgs`
- `ptz_tracker_interfaces` (action definition)
