# arm_moveit_config

Minimal MoveIt 2 configuration package for the arm simulation workspace.

This package keeps the existing fake-system baseline intact and layers MoveIt 2 on top of:

* `arm_description` for the robot model
* `arm_bringup` for the canonical bringup flow
* `arm_control` for `FollowJointTrajectory` execution through `arm_controller`

Primary launch:

```bash
ros2 launch arm_moveit_config moveit.launch.py
```

One-command demo helper:

```bash
./scripts/run_moveit_demo.sh
```

Headless smoke test:

```bash
./scripts/moveit_smoke_test.sh
```

Demo flow in RViz:

* wait for the baseline auto-home motion to finish
* open the `MotionPlanning` display
* keep planning group `arm`
* use the interactive marker on `tcp` to set a target
* click `Plan`, then `Execute`
