"""Stable factory visuals, independent of MoveIt's virtual planning states."""

import copy

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
    lamp = ((0.95, 0.08, 0.05) if phase == "stopped" else
            (0.98, 0.70, 0.06) if phase in ("planning", "executing", "perception") else
            (0.08, 0.95, 0.38))
    markers.append(Marker(
        header=Header(frame_id=scene.frame), ns="factory_signage", id=8,
        type=Marker.CUBE, action=Marker.ADD,
        pose=Pose(position=Point(x=0.88, y=0.64, z=1.09), orientation=Quaternion(w=1.0)),
        scale=Vector3(x=0.045, y=0.045, z=0.045),
        color=ColorRGBA(r=lamp[0], g=lamp[1], b=lamp[2], a=1.0)))
    return MarkerArray(markers=markers)


def environment_markers(scene):
    initial = scene.initial()
    colors = {item.id: item.color for item in initial.object_colors}
    markers = []
    for index, obj in enumerate(initial.world.collision_objects):
        # The camera and status lamp each have a single, separate visual owner.
        if obj.id in scene.dynamic_ids or obj.id == "tower_light_2" or colors[obj.id].a == 0:
            continue
        markers.append(Marker(
            header=Header(frame_id=scene.frame), ns="factory_environment", id=index,
            type=Marker.CUBE, action=Marker.ADD,
            pose=copy.deepcopy(obj.primitive_poses[0]),
            scale=Vector3(**dict(zip(("x", "y", "z"), obj.primitives[0].dimensions))),
            color=copy.deepcopy(colors[obj.id])))
    return MarkerArray(markers=markers)


def part_markers(scene, poses, colors):
    """A complete retained snapshot; missing physics bodies are explicitly removed."""
    markers = []
    for index, item in enumerate(scene.dynamic_ids):
        marker = Marker(header=Header(frame_id=scene.frame), ns="factory_parts", id=index,
                        type=Marker.CUBE, action=Marker.DELETE)
        if item in poses and item in colors:
            r, g, b = PALETTE[colors[item]]
            marker.action = Marker.ADD
            marker.pose = copy.deepcopy(poses[item])
            marker.scale = Vector3(**dict(zip(("x", "y", "z"), scene.config["object"]["size"])))
            marker.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
        markers.append(marker)
    return MarkerArray(markers=markers)
