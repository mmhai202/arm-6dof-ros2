# arm-simulation

ROS 2 Humble workspace for a simulation-first 6-DOF generic robot arm.

The project is intentionally scoped to a small Phase 1 baseline: robot description, bringup, and controller validation. The stable path today is the `ros2_control` fake system flow. A separate Gazebo path is included for ongoing simulator integration work.

## Project Architecture

The workspace is split into three packages with clear ownership:

* `arm_description`
  source of truth for the robot model, frame tree, joint limits, and `ros2_control` definition

* `arm_bringup`
  launch files for RViz, controller bringup, and Gazebo experiments

* `arm_control`
  controller configuration for `joint_state_broadcaster` and `joint_trajectory_controller`

Supporting scripts under `scripts/` provide dependency checks, smoke tests, trajectory validation, Gazebo smoke testing, and workspace cleanup.

## Baseline Model

The current baseline is a generic articulated industrial-style 6-axis arm with:

* root helper frame `base_footprint`
* main body frame `base_link`
* end-effector chain `link_6 -> flange -> tool0 -> tcp`
* `tool0` kept as a compatibility frame
* configurable `ros2_control` backend through launch/Xacro arguments

## Basic Commands

### Check dependencies

```bash
./scripts/check_dependencies.sh
```

### Build the workspace

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src
source install/setup.bash
```

### Run the baseline smoke test

```bash
./scripts/smoke_test.sh
```

### Validate the trajectory command path

```bash
./scripts/run_trajectory_demo.sh
```

### Launch in RViz

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch arm_bringup view_arm.launch.py
```

### Launch headless

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch arm_bringup view_arm.launch.py use_rviz:=false
```

### Launch in Gazebo

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch arm_bringup gazebo_arm.launch.py
```

### Run the Gazebo smoke test

```bash
./scripts/gazebo_smoke_test.sh
```

### Clean workspace artifacts

```bash
./scripts/clean_ws.sh
```
