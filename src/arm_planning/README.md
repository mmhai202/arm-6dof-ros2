# arm_planning

Project-owned planning package for the simulation workspace.

It currently provides:

* a MoveIt-free joint-space planner that reads the current joint state
* shared named-pose planning based on `arm_control/config/named_poses.yaml`
* joint-limit validation and velocity-aware segment timing from `arm_description/urdf/arm.urdf.xacro`
* workspace guard for `tcp` from `config/workspace_guard.yaml`
* multi-point `FollowJointTrajectory` execution through the existing `arm_controller`
* a long-running planning service mode for named-pose commands

## Common Commands

```bash
./scripts/run_custom_planner_demo.sh
```

```bash
./scripts/run_planning_mode.sh --moveit false
```

```bash
python3 scripts/call_planning_service.py --list-poses
python3 scripts/call_planning_service.py --pose home
python3 scripts/call_planning_service.py --pose-sequence home reach_forward inspection home
```

```bash
./scripts/custom_planning_service_smoke_test.sh
```

```bash
python3 scripts/plan_joint_trajectory.py --list-poses
```

This planner is intentionally simple: it is obstacle-free and interpolates directly in joint space without MoveIt, but it now validates target poses, waypoint timing, and `tcp` workspace bounds against project-owned robot/planner config. The service mode exposes that planner through `arm_msgs/srv/ListNamedPoses` and `arm_msgs/srv/ExecutePlan`.
