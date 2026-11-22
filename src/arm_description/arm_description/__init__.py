"""Shared robot description resources and launch configuration."""

JOINT_NAMES = tuple(f"joint_{index}" for index in range(1, 7))
GRIPPER_JOINT_NAMES = ("finger_left_joint", "finger_right_joint")
GRIPPER_TOUCH_LINKS = ("finger_left", "finger_right")
