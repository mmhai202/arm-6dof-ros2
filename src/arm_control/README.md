# arm_control

Controller configuration package for the simulation baseline.

It currently provides:

* `joint_state_broadcaster`
* `arm_controller` based on `joint_trajectory_controller`
* the installed `send_trajectory_goal.py` helper used by the baseline launch auto-home behavior
* shared named poses in `config/named_poses.yaml` for baseline demos and the custom planner mode

## Common Commands

```bash
./scripts/run_trajectory_demo.sh
```

This command launches the fake-system baseline headless and validates a representative preset pose sequence through one `FollowJointTrajectory` client session.

```bash
python3 scripts/send_trajectory_goal.py --list-poses
```
