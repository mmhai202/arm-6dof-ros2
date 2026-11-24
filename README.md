# arm-simulation

6-DOF arm simulation with ROS 2 Humble, Gazebo, MoveIt and RViz.

## Setup

Requires Miniforge/Miniconda with `conda` on `PATH`. Run from the workspace root:

```bash
./scripts/install_dependencies.sh
```

## Run

```bash
./scripts/run_moveit.sh              # Manual control in RViz
./scripts/run_moveit.sh --pick-place # Shelf-to-box pick-and-place
```

Use **Plan & Execute** for manual control, or **Run Pick & Place** / **Stop**
for the demo. Gazebo runs in the background.

Demo settings: [shelf_pick_place.yaml](src/arm_moveit_config/config/shelf_pick_place.yaml).

![Robot simulation demo](tests/demo.gif)

## Tests

```bash
source scripts/install_dependencies.sh
arm_activate_conda
source install/setup.bash
colcon test --packages-select arm_moveit_config
colcon test-result --verbose
```

## Cleanup

```bash
./scripts/clean_ws.sh  # Keep .conda/
```
