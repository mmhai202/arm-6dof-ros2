# arm_control

Controller configuration package for the simulation baseline.

It currently provides:

* `joint_state_broadcaster`
* `arm_controller` based on `joint_trajectory_controller`

## Common Commands

```bash
./scripts/run_trajectory_demo.sh
```

```bash
python3 scripts/send_trajectory_goal.py --list-poses
```
