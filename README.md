# arm-simulation

6-DOF arm simulation with ROS 2 Humble, Gazebo, MoveIt and RViz.

## Setup

Requires Miniforge/Miniconda with `conda` on `PATH`. Run from the workspace root:

```bash
./scripts/install_dependencies.sh  # Install dependencies in .conda/ and build
```

Run setup before the first launch and rerun after source changes or cleanup.
Keep the Conda environment separate from `/opt/ros`; use `ARM_CONDA_PREFIX`
to override `.conda/`.

## Run

Run from the workspace root after setup. These commands only launch the simulation:

```bash
./scripts/run_moveit.sh              # Open RViz with Gazebo physics in the background
./scripts/run_moveit.sh --pick-place # Open the shelf scene; start with the RViz button
```

Gazebo always runs without its GUI, providing physics and robot state in place of
real hardware. Use RViz to view and control the robot.

The robot starts at `home`. In RViz, drag the goal and use **Plan & Execute**.
With `--pick-place`, the scene waits for **Run Pick & Place** in the **Pick & Place**
panel. That button runs the full shelf-to-box cycle; click again after it finishes to repeat. All scripts support `--help`.

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
