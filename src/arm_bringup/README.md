# arm_bringup

Launch package for the simulation baseline.

It owns:

* RViz bringup
* controller bringup
* Gazebo launch experiments

## Common Commands

```bash
ros2 launch arm_bringup view_arm.launch.py
```

```bash
ros2 launch arm_bringup view_arm.launch.py use_rviz:=false
```

```bash
ros2 launch arm_bringup gazebo_arm.launch.py
```
