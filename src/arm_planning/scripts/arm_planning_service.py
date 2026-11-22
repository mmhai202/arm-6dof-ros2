#!/usr/bin/env python3
"""ROS executable entry point for arm_planning.service."""

from arm_planning.service import main


if __name__ == "__main__":
    raise SystemExit(main())
