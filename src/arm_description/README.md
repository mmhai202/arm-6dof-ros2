# arm_description

Robot description package for the simulation baseline.

It owns:

* URDF/Xacro
* link, joint, and frame naming
* joint limits and basic joint dynamics
* the end-effector chain `flange -> tool0 -> tcp`

## Common Commands

```bash
source /opt/ros/humble/setup.bash
xacro src/arm_description/urdf/arm.urdf.xacro > /tmp/arm.urdf
check_urdf /tmp/arm.urdf
```
