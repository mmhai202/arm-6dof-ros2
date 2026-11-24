"""Build Gazebo geometry from the same boxes MoveIt uses for planning."""

import math
import uuid
import xml.etree.ElementTree as ET

from .shelf_scene import OBJECT_ID


ENVIRONMENT_ID = "shelf_environment"


def _text(parent, tag, value):
    ET.SubElement(parent, tag).text = str(value)


def _box_link(model, obj, *, mass=None, friction=0.8, visual_suffix="", color=None):
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
        # The sensor renderer can retain an OGRE material after physics confirms
        # model deletion. New visual names prevent a duplicate-material crash
        # when the task immediately respawns the same model and link names.
        element = ET.SubElement(link, tag, name=tag + visual_suffix if tag == "visual" else tag)
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
            rgba = color or ("0.95 0.20 0.06 1" if mass is not None else (
                "0.12 0.42 0.70 1" if obj.id.startswith("box_") else "0.58 0.35 0.16 1"))
            material = ET.SubElement(element, "material")
            _text(material, "ambient", rgba)
            _text(material, "diffuse", rgba)


def gazebo_models(scene):
    """Return static fixtures and a free dynamic payload; no welds or grasp joints."""
    physics = scene.config["physics"]
    for key in ("mass", "friction", "grip_compression", "min_contact_force",
                "max_grasp_drift", "contact_timeout", "settle_time", "placement_hold_time",
                "placement_timeout", "placement_linear_speed", "placement_angular_speed"):
        if not math.isfinite(physics[key]) or physics[key] <= 0:
            raise ValueError(f"physics.{key} must be finite and positive.")
    if physics["grip_compression"] >= scene.gripper_closed:
        raise ValueError("Payload must leave enough gripper travel to squeeze before the stop.")
    if physics["placement_hold_time"] >= physics["placement_timeout"]:
        raise ValueError("physics.placement_timeout must exceed physics.placement_hold_time.")
    dynamic_ids = getattr(scene, "dynamic_ids", (OBJECT_ID,))
    roots = {name: ET.Element("sdf", version="1.6") for name in (ENVIRONMENT_ID, *dynamic_ids)}
    models = {name: ET.SubElement(root, "model", name=name) for name, root in roots.items()}
    visual_suffix = "_" + uuid.uuid4().hex[:12]
    _text(models[ENVIRONMENT_ID], "static", "true")
    for name in dynamic_ids:
        _text(models[name], "static", "false")
    initial = scene.initial()
    colors = {item.id: " ".join(str(getattr(item.color, axis)) for axis in "rgba")
              for item in initial.object_colors} if hasattr(scene, "dynamic_ids") else {}
    for obj in initial.world.collision_objects:
        if obj.id.startswith("perception_camera_"):
            continue  # The static camera model owns these boxes in Gazebo.
        if obj.id in dynamic_ids:
            _box_link(models[obj.id], obj, mass=physics["mass"], friction=physics["friction"],
                      visual_suffix=visual_suffix, color=colors.get(obj.id))
        else:
            _box_link(models[ENVIRONMENT_ID], obj, visual_suffix=visual_suffix, color=colors.get(obj.id))
    return {name: ET.tostring(root, encoding="unicode") for name, root in roots.items()}


def payload_in_box(scene, position, tolerance=0.01, *, orientation=None):
    """Check the rotated payload's bounds and its lowest point above the bottom."""
    box = scene.config["box"]
    size = scene.config["object"]["size"]
    if orientation is None:
        half = [value / 2 for value in size]
    else:
        q = [getattr(orientation, axis) for axis in "xyzw"]
        norm = math.sqrt(sum(value * value for value in q))
        if not math.isfinite(norm) or norm < 1e-12:
            return False
        x, y, z, w = (value / norm for value in q)
        rotation = (
            (1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)),
            (2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)),
            (2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)),
        )
        half = [sum(abs(value) * extent / 2 for value, extent in zip(row, size))
                for row in rotation]
    if not all(math.isfinite(getattr(position, axis)) for axis in "xyz"):
        return False
    for index, axis in enumerate(("x", "y")):
        if (abs(getattr(position, axis) - box["center_xy"][index]) + half[index]
                > box["size_xy"][index] / 2 - box["thickness"]):
            return False
    bottom = box.get("base_z", 0.0)
    return (abs(position.z - half[2] - bottom - box["thickness"]) <= tolerance
            and position.z + half[2] <= bottom + box["height"] + tolerance)


def payload_is_settled(scene, pose, twist):
    if pose is None or twist is None:
        return False
    if not payload_in_box(scene, pose.position, orientation=pose.orientation):
        return False
    for vector, key in ((twist.linear, "placement_linear_speed"),
                        (twist.angular, "placement_angular_speed")):
        speed = math.sqrt(sum(getattr(vector, axis)**2 for axis in "xyz"))
        if not math.isfinite(speed) or speed > scene.config["physics"][key]:
            return False
    return True
