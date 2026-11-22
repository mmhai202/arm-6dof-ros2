# arm-simulation

6-DOF arm simulation with ROS 2 Humble, RViz and fake hardware.

## Setup and run

Requires Miniforge/Miniconda with `conda` on `PATH`. Run from the workspace root:

```bash
./scripts/install_dependencies.sh  # Install all dependencies in .conda/ and build
```

Start MoveIt:

```bash
./scripts/run_moveit.sh # MoveIt: drag the goal in RViz, then Plan & Execute
```

Fake hardware starts upright at `home`, including in `--scene-only` mode.
The MoveIt launcher supports `--headless`.
Run scripts only check and launch. Missing dependencies or build artifacts are reported.
Rerun the installer after source changes or cleanup.

Keep this Conda environment separate from `/opt/ros`.

## Demo: pick from a cabinet shelf into a floor box

```bash
colcon build --packages-select arm_moveit_config --symlink-install  # Rebuild in the activated environment
./scripts/run_moveit.sh --pick-place             # Pick from shelf and place in floor box
./scripts/run_moveit.sh --scene-only             # Show scene only
./scripts/run_moveit.sh --plan-only              # Plan without moving
./scripts/run_moveit.sh --pick-place --headless  # Run without RViz
ros2 run arm_moveit_config shelf_pick_place.py --mode execute  # Repeat in an active MoveIt session
```

![Robot simulation demo](tests/demo.gif)

## Cleanup and tests

```bash
./scripts/clean_ws.sh                       # Remove build artifacts; keep .conda/
python3 -m unittest discover -s tests -v     # With environment and workspace sourced
```
