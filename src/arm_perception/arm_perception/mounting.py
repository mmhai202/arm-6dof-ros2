"""Shared camera and roof bracket boxes in the configured robot base frame."""

import math


def camera_fixtures(config):
    fixtures = [{
        **part, "name": "perception_camera_" + part["name"],
        "rpy": [0.0, 0.0, 0.0], "color": [0.28, 0.32, 0.38],
    } for part in config["mount"]]
    camera = config["camera"]
    _, pitch, yaw = camera["rpy"]
    forward = [math.cos(yaw)*math.cos(pitch), math.sin(yaw)*math.cos(pitch), -math.sin(pitch)]
    fixtures.append({
        "name": "perception_camera_housing", "size": [0.06, 0.09, 0.045],
        "center": [p - 0.035*d for p, d in zip(camera["position"], forward)],
        "rpy": list(camera["rpy"]), "color": [0.28, 0.32, 0.38],
    })
    return fixtures
