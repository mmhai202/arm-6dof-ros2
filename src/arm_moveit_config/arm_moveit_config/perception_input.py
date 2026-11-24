"""Validate camera targets and keep simulator spawning separate from grasping."""

from collections import deque
import math

import numpy as np


PERCEPTION_PARAMETERS = {
    "use_perception": "enabled", "randomize_payload": "randomize_spawn",
    "perception_seed": "spawn_seed",
}


def validate_perception_config(config):
    for key in ("timeout", "max_age", "max_spread", "clock_tolerance"):
        if not math.isfinite(config[key]) or config[key] <= 0:
            raise ValueError(f"perception.{key} must be finite and positive.")
    count = config["stable_samples"]
    if isinstance(count, bool) or not isinstance(count, int) or count < 2:
        raise ValueError("perception.stable_samples must be an integer of at least two.")


def validate_pick_center(scene, center):
    center = np.asarray(center, dtype=float)
    if center.shape != (3,) or not np.isfinite(center).all():
        raise ValueError("Object position must contain three finite coordinates.")
    shelf = scene.config["shelf"]
    size = scene.config["object"]["size"]
    for axis in (0, 1):
        if abs(center[axis] - shelf["center_xy"][axis]) + size[axis]/2 > shelf["size_xy"][axis]/2 - 0.01:
            raise ValueError("Detected object is outside the supported shelf region.")
    expected_z = shelf["surface_z"] + size[2]/2
    if abs(center[2] - expected_z) > 0.012:
        raise ValueError("Detected object is not resting on the shelf.")
    return center


def sample_spawn_center(scene, rng):
    bounds = np.asarray(scene.config["perception"]["spawn_xy_bounds"], dtype=float)
    if bounds.shape != (2, 2) or not np.isfinite(bounds).all() or np.any(bounds[:, 1] < bounds[:, 0]):
        raise ValueError("perception.spawn_xy_bounds must contain ordered X and Y intervals.")
    z = scene.config["shelf"]["surface_z"] + scene.config["object"]["size"][2]/2
    # Validate every corner, not just the sampled point.
    for x in bounds[0]:
        for y in bounds[1]:
            validate_pick_center(scene, [x, y, z])
    return [rng.uniform(*bounds[0]), rng.uniform(*bounds[1]), z]


class DetectionWindow:
    def __init__(self, scene, after):
        self.scene = scene
        self.config = scene.config["perception"]
        validate_perception_config(self.config)
        self.after = after
        self.samples = deque(maxlen=self.config["stable_samples"])
        self.last_stamp = -math.inf
        self.reason = "No camera pose received."

    def add(self, message, now):
        try:
            stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            if message.header.frame_id != self.scene.frame:
                raise ValueError("Camera pose is in the wrong coordinate frame.")
            if stamp <= self.after or not -self.config["clock_tolerance"] <= now - stamp <= self.config["max_age"]:
                raise ValueError("Camera pose is stale or predates this observation.")
            if stamp <= self.last_stamp:
                return None  # A repeated timestamp cannot count toward stability.
            self.last_stamp = stamp
            p = message.pose.position
            center = validate_pick_center(self.scene, [p.x, p.y, p.z])
            q = message.pose.orientation
            if not np.isfinite([q.x, q.y, q.z, q.w]).all() or abs(q.w) < 0.999 or max(abs(q.x), abs(q.y), abs(q.z)) > 0.01:
                raise ValueError("Only axis-aligned payloads are supported.")
            if self.samples and stamp - self.samples[-1][0] > self.config["max_age"]:
                self.samples.clear()
            self.samples.append((stamp, center))
            if len(self.samples) < self.samples.maxlen:
                self.reason = "Waiting for consecutive camera samples."
                return None
            points = np.array([point for _, point in self.samples])
            if np.max(np.ptp(points, axis=0)) > self.config["max_spread"]:
                self.reason = "Camera target is moving or unstable."
                return None
            self.reason = "Stable camera target."
            return np.median(points, axis=0).tolist()
        except ValueError as error:
            self.samples.clear()
            self.reason = str(error)
            return None
