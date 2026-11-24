"""Serialize local shelf tasks and their asynchronous planning-scene updates."""

import fcntl
import os
from pathlib import Path
import tempfile
import uuid


class SceneOwnership:
    def __init__(self, path=None):
        if path is None:
            domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
            path = Path(tempfile.gettempdir()) / f"arm_shelf_scene_{os.getuid()}_{domain}.lock"
        # Keep this file: unlinking it could give contenders different lock inodes.
        self.stream = Path(path).open("a+", encoding="ascii")
        self.token = uuid.uuid4().hex
        self.held = False
        self.superseded = False
        self.pending = False

    def _acquire(self):
        if not self.held:
            try:
                fcntl.flock(self.stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                return False
            self.held = True
        return True

    def claim(self):
        """Claim a task only after the previous writer's update is acknowledged."""
        if not self._acquire():
            return False
        self.stream.seek(0)
        if self.stream.read().startswith("unacknowledged:"):
            self.release()
            raise RuntimeError(
                "Previous planning-scene update was not acknowledged. Stop all simulation "
                f"processes, restart MoveIt, then clear {self.stream.name} before retrying.")
        self._write(self.token)
        self.pending = False
        return True

    def _write(self, value):
        self.stream.seek(0)
        self.stream.truncate()
        self.stream.write(value)
        self.stream.flush()

    def quarantine(self):
        """Preserve an uncertain write across process exit; never unlink the lock."""
        if self.held:
            self.pending = True
            self._write(f"unacknowledged:{self.token}")

    def acknowledge(self):
        if self.held:
            self.stream.seek(0)
            if self.stream.read() == f"unacknowledged:{self.token}":
                self._write(self.token)
                self.pending = False

    def resume(self):
        """An idle owner may synchronize until another task claims the scene."""
        if self.superseded or not self._acquire():
            return False
        self.stream.seek(0)
        if self.stream.read() not in (self.token, f"unacknowledged:{self.token}"):
            self.superseded = True
            self.release()
            return False
        return True

    def release(self):
        if self.held:
            fcntl.flock(self.stream, fcntl.LOCK_UN)
            self.held = False

    def close(self):
        self.stream.close()
        self.held = False
