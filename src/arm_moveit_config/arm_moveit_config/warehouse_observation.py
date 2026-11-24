"""Associate fresh RGB-D observations with occupied fixture slots."""

from collections import deque
import math

import numpy as np


class StockObservation:
    def __init__(self, scene, after):
        self.scene, self.after = scene, after
        self.samples = deque(maxlen=scene.config["perception"]["stable_samples"])
        self.last_stamp = -math.inf
        self.reason = "Waiting for the stock camera."

    def add(self, message, now):
        try:
            config = self.scene.config["perception"]
            stamp = float(message["stamp"])
            if (message["frame"] != self.scene.frame or not math.isfinite(stamp)
                    or stamp <= self.after
                    or not -config["clock_tolerance"] <= now - stamp <= config["max_age"]):
                raise ValueError("Stock camera frame or timestamp is invalid/stale.")
            if stamp <= self.last_stamp:
                return None
            self.last_stamp = stamp
            detections = message["objects"]
            occupied = {item: slot for item, slot in self.scene.locations.items() if slot.startswith("stock_")}
            if len(detections) != len(occupied):
                raise ValueError(f"Stock camera sees {len(detections)}/{len(occupied)} parts.")
            matched, used = {}, set()
            for item, slot in occupied.items():
                center = np.asarray(self.scene.slot_centers[slot], dtype=float)
                candidates = []
                for index, detection in enumerate(detections):
                    xyz = np.asarray(detection["center"], dtype=float)
                    if xyz.shape != (3,) or not np.isfinite(xyz).all():
                        raise ValueError("Stock camera coordinates must be finite XYZ values.")
                    if detection["color"] not in ("red", "blue", "green"):
                        raise ValueError("Unknown stock color.")
                    if index not in used and np.linalg.norm(xyz[:2] - center[:2]) < self.scene.warehouse["association_distance"]:
                        candidates.append(index)
                if len(candidates) != 1:
                    raise ValueError("Missing or ambiguous stock identity at a fixture slot.")
                index = candidates[0]
                detection = detections[index]
                if abs(detection["center"][2] - center[2]) > 0.012:
                    raise ValueError("Stock is not resting on its deck.")
                matched[item] = (detection["color"], detection["center"])
                used.add(index)
            if self.samples and stamp - self.samples[-1][0] > config["max_age"]:
                self.samples.clear()
            self.samples.append((stamp, matched))
            if len(self.samples) < self.samples.maxlen:
                self.reason = "Waiting for consecutive stock observations."
                return None
            result = {}
            for item in occupied:
                if len({sample[item][0] for _, sample in self.samples}) != 1:
                    raise ValueError("Stock color identity is unstable.")
                points = np.array([sample[item][1] for _, sample in self.samples])
                if np.max(np.ptp(points, axis=0)) > config["max_spread"]:
                    raise ValueError("Stock camera positions are unstable.")
                result[item] = (matched[item][0], np.median(points, axis=0).tolist())
            return result
        except (ValueError, TypeError, KeyError) as error:
            self.samples.clear()
            self.reason = str(error)
            return None
