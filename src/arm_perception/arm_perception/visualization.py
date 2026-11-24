"""Camera housing and field-of-view markers attached to the sensor's TF frames."""

import math

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import quaternion_from_rpy
from .mounting import camera_details, camera_fixtures


def camera_markers(config):
    markers = []

    def marker(name, kind, frame="camera_link"):
        result = Marker(ns=name, id=0, type=kind, action=Marker.ADD)
        result.header.frame_id = frame
        result.pose.orientation.w = 1.0
        result.color.a = 1.0
        markers.append(result)
        return result

    for fixture in camera_fixtures(config):
        body = marker(fixture["name"], Marker.CUBE, config["frame"])
        p, q = body.pose.position, body.pose.orientation
        p.x, p.y, p.z = map(float, fixture["center"])
        q.x, q.y, q.z, q.w = quaternion_from_rpy(*fixture["rpy"])
        body.scale.x, body.scale.y, body.scale.z = map(float, fixture["size"])
        body.color.r, body.color.g, body.color.b = map(float, fixture["color"])

    for part in camera_details():
        detail = marker(part["name"], Marker.CYLINDER if part["shape"] == "cylinder" else Marker.CUBE)
        p, q = detail.pose.position, detail.pose.orientation
        p.x, p.y, p.z = part["center"]
        q.x, q.y, q.z, q.w = quaternion_from_rpy(*part["rpy"])
        detail.scale.x, detail.scale.y, detail.scale.z = part["size"]
        detail.color.r, detail.color.g, detail.color.b = part["color"]

    # Remove the old stereo-style lenses if RViz remains open during a restart.
    for side in ("left", "right"):
        marker(f"camera_lens_{side}", Marker.CYLINDER).action = Marker.DELETE

    # Draw the first 60 cm of the viewing volume so it stays readable around
    # the shelf. The sensor's full near/far clipping range remains unchanged.
    camera = config["camera"]
    distances = [camera["near"], min(camera["far"], 0.60)]
    planes = []
    for depth in distances:
        half_width = depth * math.tan(camera["horizontal_fov"]/2)
        half_height = half_width * camera["height"] / camera["width"]
        planes.append([Point(x=x, y=y, z=float(depth)) for x, y in (
            (-half_width, -half_height), (half_width, -half_height),
            (half_width, half_height), (-half_width, half_height),
        )])
    frustum = marker("camera_view", Marker.LINE_LIST, "camera_optical_frame")
    frustum.scale.x = 0.002
    frustum.color.r, frustum.color.g, frustum.color.b, frustum.color.a = 0.1, 0.8, 1.0, 0.45
    for index in range(4):
        following = (index + 1) % 4
        for plane in planes:
            frustum.points.extend([plane[index], plane[following]])
        frustum.points.extend([planes[0][index], planes[1][index]])

    return MarkerArray(markers=markers)
