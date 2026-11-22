# arm-simulation

6-DOF arm simulation with ROS 2 Humble, RViz and fake hardware.

## Setup and run

Requires Miniforge/Miniconda with `conda` on `PATH`. Run from the workspace root:

```bash
./scripts/install_dependencies.sh  # Install all dependencies in .conda/ and build
```

Choose one mode:

```bash
./scripts/run_moveit_demo.sh    # MoveIt: drag the goal in RViz, then Plan & Execute
./scripts/run_planning_mode.sh # Custom planner: send named poses from a terminal
```

Both support `--headless`; custom planning also supports `--auto-home`.
Run scripts only check and launch. Missing dependencies or build artifacts are reported.
Rerun the installer after source changes or cleanup.

## Custom planning commands

After starting custom planning, open another terminal:

```bash
source scripts/install_dependencies.sh  # Load helpers only
arm_activate_conda
source install/setup.bash
ros2 run arm_planning call_planning_service.py --list-poses
ros2 run arm_planning call_planning_service.py --pose-sequence home reach_forward inspection home
```

Keep this Conda environment separate from `/opt/ros`.

## Cleanup and tests

```bash
./scripts/clean_ws.sh                       # Remove build artifacts; keep .conda/
python3 -m unittest discover -s tests -v     # With environment and workspace sourced
```
