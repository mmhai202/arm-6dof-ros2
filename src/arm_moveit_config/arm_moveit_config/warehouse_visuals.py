"""Stable factory visuals, independent of MoveIt's virtual planning states."""

import copy

import numpy as np

from arm_perception.geometry import transform_matrix
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from .warehouse_scene import PALETTE


def factory_markers(scene, phase, stage=""):
    # Keep the old IDs so an existing RViz session also loses the retired labels.
    markers = [Marker(header=Header(frame_id=scene.frame), ns="factory_signage", id=index, action=Marker.DELETE)
               for index in range(7)]
    markers.append(Marker(
        header=Header(frame_id=scene.frame), ns="factory_signage", id=7,
        type=Marker.TEXT_VIEW_FACING, action=Marker.ADD,
        pose=Pose(position=Point(x=0.0, y=-0.85, z=0.04), orientation=Quaternion(w=1.0)),
        scale=Vector3(z=0.033), color=ColorRGBA(r=0.88, g=0.95, b=0.97, a=1.0),
        text=phase.upper()))
    # Retire the phase-colored overlay; all three lamps belong to the static scene.
    markers.append(Marker(
        header=Header(frame_id=scene.frame), ns="factory_signage", id=8,
        action=Marker.DELETE))
    return MarkerArray(markers=markers)


def environment_markers(scene):
    initial = scene.initial()
    colors = {item.id: item.color for item in initial.object_colors}
    markers = []
    for index, obj in enumerate(initial.world.collision_objects):
        # The camera has its own visual owner; the tower lamps stay static here.
        if obj.id in scene.dynamic_ids or colors[obj.id].a == 0:
            continue
        markers.append(Marker(
            header=Header(frame_id=scene.frame), ns="factory_environment", id=index,
            type=Marker.CUBE, action=Marker.ADD,
            pose=copy.deepcopy(obj.primitive_poses[0]),
            scale=Vector3(**dict(zip(("x", "y", "z"), obj.primitives[0].dimensions))),
            color=copy.deepcopy(colors[obj.id])))
    return MarkerArray(markers=markers)


def wrist_relative_pose(wrist, payload):
    """Express a measured body pose in the wrist from the same physics sample."""
    reference = np.array([getattr(wrist.orientation, axis) for axis in "xyzw"])
    orientation = np.array([getattr(payload.orientation, axis) for axis in "xyzw"])
    reference /= np.linalg.norm(reference)
    orientation /= np.linalg.norm(orientation)
    transform = transform_matrix([getattr(wrist.position, axis) for axis in "xyz"], reference)
    position = transform[:3, :3].T @ (
        np.array([getattr(payload.position, axis) for axis in "xyz"]) - transform[:3, 3])
    # conjugate(reference) * orientation
    xyz = (reference[3] * orientation[:3] - orientation[3] * reference[:3]
           - np.cross(reference[:3], orientation[:3]))
    w = reference[3] * orientation[3] + np.dot(reference[:3], orientation[:3])
    return Pose(position=Point(**dict(zip("xyz", map(float, position)))),
                orientation=Quaternion(**dict(zip("xyzw", map(float, [*xyz, w])))))


def part_markers(scene, poses, colors, *, held_item=None, wrist_pose=None):
    """A complete retained snapshot; missing physics bodies are explicitly removed."""
    markers = []
    for index, item in enumerate(scene.dynamic_ids):
        marker = Marker(header=Header(frame_id=scene.frame), ns="factory_parts", id=index,
                        type=Marker.CUBE, action=Marker.DELETE)
        if item in poses and item in colors:
            r, g, b = PALETTE[colors[item]]
            marker.action = Marker.ADD
            marker.pose = copy.deepcopy(poses[item])
            if item == held_item and wrist_pose is not None:
                # RViz follows the same live wrist TF as RobotModel between
                # physics messages. Recompute the offset on every sample so
                # physical slip and rotation remain visible.
                marker.header.frame_id = "link_6"
                marker.frame_locked = True
                marker.pose = wrist_relative_pose(wrist_pose, poses[item])
            marker.scale = Vector3(**dict(zip(("x", "y", "z"), scene.config["object"]["size"])))
            marker.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
        markers.append(marker)
    return MarkerArray(markers=markers)
