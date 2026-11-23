# arm-simulation

6-DOF arm simulation with ROS 2 Humble, Gazebo, MoveIt and RViz.

## Setup and run

Requires Miniforge/Miniconda with `conda` on `PATH`. Run from the workspace root:

```bash
./scripts/install_dependencies.sh  # Install dependencies in .conda/ and build
./scripts/run_moveit.sh                        # Open Gazebo and RViz
./scripts/run_moveit.sh --headless             # Gazebo in the background; RViz stays open
./scripts/run_moveit.sh --pick-place           # Pick from shelf and place in floor box
./scripts/run_moveit.sh --pick-place --headless
```

The robot starts at `home`. In RViz, drag the goal and use **Plan & Execute**.
Press Ctrl+C to stop. All scripts support `--help`.

Rerun the installer after source changes or cleanup. Keep the Conda environment
separate from `/opt/ros`; use `ARM_CONDA_PREFIX` to override `.conda/`.

Demo settings: [shelf_pick_place.yaml](src/arm_moveit_config/config/shelf_pick_place.yaml).

![Robot simulation demo](tests/demo.gif)

## Cleanup and tests

```bash
source scripts/install_dependencies.sh  # Load helpers only
arm_activate_conda
source install/setup.bash
python3 -m unittest discover -s tests -v
./scripts/clean_ws.sh  # Remove build/, install/, log/ and Python caches; keep .conda/
```

Integration tests require a disposable simulation with the same `ROS_DOMAIN_ID`.
Enable with `ARM_RUN_MOVEIT_TESTS=1`, `ARM_RUN_GAZEBO_TESTS=1`, or
`ARM_RUN_GAZEBO_GRASP_TESTS=1`; grasp tests need a session without `--pick-place`.
