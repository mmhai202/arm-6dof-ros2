#!/usr/bin/env python3
"""Run the MoveIt shelf pick-and-place scenario."""

import sys

from arm_moveit_config.shelf_task import main


if __name__ == "__main__":
    sys.exit(main())
