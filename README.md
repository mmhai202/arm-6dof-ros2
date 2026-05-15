# arm-simulation

ROS 2 Humble workspace for a simulation 6-DOF generic robot arm.

The project provides robot description, bringup, controller validation, and an optional MoveIt 2 planning path on top of the current `ros2_control` fake-system flow. The Gazebo Classic path is kept only as an optional integration reference.

## Project Architecture

The workspace is split into six packages with clear ownership:

* `arm_description`
  source of truth for the robot model, CAD-derived UR7e visuals, frame tree, joint limits, and `ros2_control` definition

* `arm_bringup`
  launch files for RViz, controller bringup, and Gazebo experiments

* `arm_control`
  controller configuration for `joint_state_broadcaster` and `joint_trajectory_controller`

* `arm_planning`
  project-owned joint-space planning and execution that does not depend on MoveIt

* `arm_msgs`
  custom service interfaces for the project-owned planning API

* `arm_moveit_config`
  minimal MoveIt 2 SRDF, planning config, controller mapping, and launch wiring for the existing baseline

## Quick Start

Run from the workspace root directory.

Optional cleanup before a fresh rebuild:

```bash
./scripts/clean_ws.sh
```

### 1) Custom Planning Mode (No MoveIt)

Start planning mode (this script runs dependency check and build):

```bash
./scripts/run_planning_mode.sh
```

Options:

```bash
./scripts/run_planning_mode.sh --headless
./scripts/run_planning_mode.sh --auto-home
```

Call the planning service from another shell:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run arm_planning call_planning_service.py --list-poses
ros2 run arm_planning call_planning_service.py --pose home
ros2 run arm_planning call_planning_service.py --pose-sequence home reach_forward inspection home
```

### 2) MoveIt Mode

Start MoveIt demo (this script runs MoveIt dependency check and build):

```bash
./scripts/run_moveit_demo.sh
```

Headless mode:

```bash
./scripts/run_moveit_demo.sh --headless
```

### UR7e CAD Visual Pipeline

Regenerate the normalized UR7e visual meshes from `UR5eUR7e.step/UR7e.step`:

```bash
python3 src/arm_description/scripts/prepare_ur7e.py --force-export
```

Current CAD artifacts live under `src/arm_description/meshes/ur7e/`:

* `local/`
  link-local meshes used directly by RViz / MoveIt

* `raw/`
  direct STEP component exports kept for inspection and regeneration

* `ur7e_manifest.yaml`
  component-to-link mapping and normalization metadata

Key top-level scripts:

* `run_planning_mode.sh`
  launch custom planning mode on top of the fake-system baseline

* `run_moveit_demo.sh`
  build and launch MoveIt demo on top of the same baseline

* `clean_ws.sh`
  remove `build/`, `install/`, `log/`, and Python cache

* `check_dependencies.sh`
  verify system commands and ROS packages for baseline, MoveIt, or Gazebo
