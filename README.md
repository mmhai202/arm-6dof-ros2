# arm-simulation

ROS 2 Humble workspace for a simulation 6-DOF generic robot arm.

The project provides robot description, bringup, controller validation, and an optional MoveIt 2 planning path on top of the current `ros2_control` fake-system flow. The Gazebo Classic path is kept only as an optional integration reference.

## Project Architecture

The workspace is split into four packages with clear ownership:

* `arm_description`
  source of truth for the robot model, frame tree, joint limits, and `ros2_control` definition

* `arm_bringup`
  launch files for RViz, controller bringup, and Gazebo experiments

* `arm_control`
  controller configuration for `joint_state_broadcaster` and `joint_trajectory_controller`

* `arm_moveit_config`
  minimal MoveIt 2 SRDF, planning config, controller mapping, and launch wiring for the existing baseline

## Quick Start

Run the main demo script:

```bash
./scripts/run_moveit_demo.sh
```

This script is the recommended entrypoint for the current project. It checks MoveIt dependencies, cleans the workspace, rebuilds it, launches the fake-system baseline together with MoveIt, and opens the RViz Motion Planning workflow for planning and execution through `arm_controller`.
