#!/usr/bin/env python3
"""Run the physical Gazebo shelf task and maintain the planning scene."""

import sys

from arm_moveit_config.gazebo_task import main


if __name__ == "__main__":
    sys.exit(main())
