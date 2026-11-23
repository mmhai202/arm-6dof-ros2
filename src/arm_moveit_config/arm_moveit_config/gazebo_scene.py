"""Build Gazebo geometry from the same boxes MoveIt uses for planning."""

import math
import xml.etree.ElementTree as ET

from .shelf_scene import OBJECT_ID


ENVIRONMENT_ID = "shelf_environment"


def _text(parent, tag, value):
    ET.SubElement(parent, tag).text = str(value)


def _box_link(model, obj, *, mass=None, friction=0.8):
    size = obj.primitives[0].dimensions
    pose = obj.primitive_poses[0].position
    link = ET.SubElement(model, "link", name=obj.id)
    _text(link, "pose", "0 0 0 0 0 0" if mass is not None else f"{pose.x} {pose.y} {pose.z} 0 0 0")
    if mass is not None:
        inertial = ET.SubElement(link, "inertial")
        _text(inertial, "mass", mass)
        inertia = ET.SubElement(inertial, "inertia")
        x, y, z = size
        for tag, value in (("ixx", mass * (y*y + z*z) / 12),
                           ("iyy", mass * (x*x + z*z) / 12),
                           ("izz", mass * (x*x + y*y) / 12),
                           ("ixy", 0), ("ixz", 0), ("iyz", 0)):
            _text(inertia, tag, value)
    for tag in ("collision", "visual"):
        element = ET.SubElement(link, tag, name=tag)
        box = ET.SubElement(ET.SubElement(element, "geometry"), "box")
        _text(box, "size", " ".join(map(str, size)))
        if tag == "collision":
            surface = ET.SubElement(element, "surface")
            ode = ET.SubElement(ET.SubElement(surface, "friction"), "ode")
            for key in ("mu", "mu2"):
                _text(ode, key, friction)
            contact = ET.SubElement(ET.SubElement(surface, "contact"), "ode")
            for key, value in (("kp", 100000), ("kd", 100), ("max_vel", 0.1), ("min_depth", 0.0005)):
                _text(contact, key, value)
        else:
            color = "0.95 0.20 0.06 1" if mass is not None else (
                "0.12 0.42 0.70 1" if obj.id.startswith("box_") else "0.58 0.35 0.16 1")
            material = ET.SubElement(element, "material")
            _text(material, "ambient", color)
            _text(material, "diffuse", color)


def gazebo_models(scene):
    """Return static fixtures and a free dynamic payload; no welds or grasp joints."""
    physics = scene.config["physics"]
    for key in ("mass", "friction", "grip_compression", "min_contact_force",
                "max_grasp_drift", "contact_timeout", "settle_time"):
        if not math.isfinite(physics[key]) or physics[key] <= 0:
            raise ValueError(f"physics.{key} must be finite and positive.")
    if physics["grip_compression"] >= scene.gripper_closed:
        raise ValueError("Payload must leave enough gripper travel to squeeze before the stop.")
    roots = {name: ET.Element("sdf", version="1.6") for name in (ENVIRONMENT_ID, OBJECT_ID)}
    models = {name: ET.SubElement(root, "model", name=name) for name, root in roots.items()}
    _text(models[ENVIRONMENT_ID], "static", "true")
    _text(models[OBJECT_ID], "static", "false")
    for obj in scene.initial().world.collision_objects:
        if obj.id == OBJECT_ID:
            _box_link(models[OBJECT_ID], obj, mass=physics["mass"], friction=physics["friction"])
        else:
            _box_link(models[ENVIRONMENT_ID], obj)
    return {name: ET.tostring(root, encoding="unicode") for name, root in roots.items()}


def payload_in_box(scene, position, tolerance=0.01):
    box = scene.config["box"]
    size = scene.config["object"]["size"]
    for index, axis in enumerate(("x", "y")):
        if (abs(getattr(position, axis) - box["center_xy"][index]) + size[index] / 2
                > box["size_xy"][index] / 2 - box["thickness"]):
            return False
    return abs(position.z - (box["thickness"] + size[2] / 2)) <= tolerance
