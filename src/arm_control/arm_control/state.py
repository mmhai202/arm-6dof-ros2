"""Complete, consistently ordered joint-state snapshots for CLI and service nodes."""

import threading
import time

from arm_description import JOINT_NAMES
import rclpy
from sensor_msgs.msg import JointState

from .poses import joint_positions


class JointStateBuffer:
    def __init__(self, node, *, callback_group=None):
        self._node = node
        self._positions = None
        self._ready = threading.Event()
        self._subscription = node.create_subscription(
            JointState, "/joint_states", self._receive, 10, callback_group=callback_group,
        )

    def _receive(self, message):
        by_name = dict(zip(message.name, message.position))
        if not all(name in by_name for name in JOINT_NAMES):
            return
        try:
            self._positions = joint_positions(by_name[name] for name in JOINT_NAMES)
        except ValueError:
            return
        self._ready.set()

    def wait_for_complete_state(self, timeout, *, spin=False):
        if spin:
            deadline = time.monotonic() + timeout
            while not self._ready.is_set() and rclpy.ok():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                rclpy.spin_once(self._node, timeout_sec=min(0.1, remaining))
        else:
            self._ready.wait(timeout)
        return list(self._positions) if self._positions is not None else None
