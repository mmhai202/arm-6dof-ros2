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
        "rpy": list(camera["rpy"]), "color": [0.58, 0.62, 0.67],
    })
    return fixtures


def camera_details():
    """Visual trim in camera_link: one central lens facing along positive X.

    All trim stays inside the 5 cm near clip plane of both camera presets.
    These small cosmetic parts do not add collision geometry.
    """
    return [{
        "name": name, "shape": shape, "center": center, "size": size, "color": color,
        "rpy": [0.0, math.pi/2, 0.0] if shape == "cylinder" else [0.0, 0.0, 0.0],
    } for name, shape, center, size, color in (
        ("camera_faceplate", "box", [-0.003, 0.0, 0.0], [0.004, 0.084, 0.041],
         [0.07, 0.08, 0.10]),
        ("camera_lens_mount", "cylinder", [0.0, 0.0, 0.0], [0.036, 0.036, 0.010],
         [0.40, 0.44, 0.49]),
        ("camera_lens_barrel", "cylinder", [0.006, 0.0, 0.0], [0.029, 0.029, 0.020],
         [0.035, 0.04, 0.05]),
        ("camera_lens_rim", "cylinder", [0.015, 0.0, 0.0], [0.032, 0.032, 0.004],
         [0.16, 0.18, 0.21]),
        ("camera_lens_glass", "cylinder", [0.0172, 0.0, 0.0], [0.024, 0.024, 0.0008],
         [0.025, 0.12, 0.19]),
        ("camera_status_led", "box", [-0.0005, -0.032, 0.012], [0.001, 0.006, 0.003],
         [0.15, 0.85, 0.35]),
    )]
