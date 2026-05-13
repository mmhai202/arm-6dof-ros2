# arm_bringup

Launch package for the simulation baseline.

It owns:

* canonical fake-system bringup in RViz or headless mode
* controller bringup
* the baseline runtime used by both the custom planner path and the MoveIt path
* Gazebo Classic launch experiments as an optional path

## Common Commands

```bash
ros2 launch arm_bringup view_arm.launch.py
```

`view_arm.launch.py` is the canonical Phase 1 launch path. It sends a short `home` goal automatically after controller activation. Disable that behavior with:

```bash
ros2 launch arm_bringup view_arm.launch.py auto_home:=false
```

```bash
ros2 launch arm_bringup view_arm.launch.py use_rviz:=false
```

```bash
ros2 launch arm_bringup gazebo_arm.launch.py
```
