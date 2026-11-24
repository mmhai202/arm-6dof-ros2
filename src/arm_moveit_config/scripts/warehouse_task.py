#!/usr/bin/env python3
from arm_moveit_config.gazebo_task import main
from arm_moveit_config.warehouse_task import WarehouseTask

if __name__ == "__main__":
    raise SystemExit(main(task_factory=WarehouseTask))
