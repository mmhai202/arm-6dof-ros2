"""Image-only cuboid localization; no simulator object states are used here."""

import math
import xml.etree.ElementTree as ET

import cv2
import numpy as np

from .mounting import camera_details, camera_fixtures


def quaternion_from_rpy(roll, pitch, yaw):
    cr, cp, cy = (math.cos(v / 2) for v in (roll, pitch, yaw))
    sr, sp, sy = (math.sin(v / 2) for v in (roll, pitch, yaw))
    return (sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy)


def transform_matrix(translation, quaternion):
    q = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("Invalid camera transform quaternion.")
    x, y, z, w = q / norm
    result = np.eye(4)
    result[:3, :3] = [
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ]
    result[:3, 3] = translation
    if not np.isfinite(result).all():
        raise ValueError("Invalid camera transform translation.")
    return result


def camera_sdf(config):
    """Use the same extrinsics for the Gazebo sensor and its static TF."""
    camera = config["camera"]
    root = ET.Element("sdf", version="1.6")
    model = ET.SubElement(root, "model", name="perception_camera")
    ET.SubElement(model, "static").text = "true"
    # SpawnEntity supplies the extrinsics as initial_pose.
    ET.SubElement(model, "pose").text = "0 0 0 0 0 0"
    link = ET.SubElement(model, "link", name="camera_link")
    base_to_camera = np.linalg.inv(transform_matrix(camera["position"], quaternion_from_rpy(*camera["rpy"])))
    for fixture in camera_fixtures(config):
        local = base_to_camera @ transform_matrix(fixture["center"], quaternion_from_rpy(*fixture["rpy"]))
        rpy = [math.atan2(local[2, 1], local[2, 2]),
               math.atan2(-local[2, 0], math.hypot(local[0, 0], local[1, 0])),
               math.atan2(local[1, 0], local[0, 0])]
        for kind in ("visual", "collision"):
            element = ET.SubElement(link, kind, name=fixture["name"])
            ET.SubElement(element, "pose").text = " ".join(map(str, [*local[:3, 3], *rpy]))
            box = ET.SubElement(ET.SubElement(element, "geometry"), "box")
            ET.SubElement(box, "size").text = " ".join(map(str, fixture["size"]))
            if kind == "visual":
                material = ET.SubElement(element, "material")
                color = " ".join(map(str, [*fixture["color"], 1.0]))
                ET.SubElement(material, "ambient").text = color
                ET.SubElement(material, "diffuse").text = color
    for part in camera_details():
        visual = ET.SubElement(link, "visual", name=part["name"])
        ET.SubElement(visual, "pose").text = " ".join(map(str, [*part["center"], *part["rpy"]]))
        shape = ET.SubElement(ET.SubElement(visual, "geometry"), part["shape"])
        if part["shape"] == "cylinder":
            ET.SubElement(shape, "radius").text = str(part["size"][0] / 2)
            ET.SubElement(shape, "length").text = str(part["size"][2])
        else:
            ET.SubElement(shape, "size").text = " ".join(map(str, part["size"]))
        material = ET.SubElement(visual, "material")
        color = " ".join(map(str, [*part["color"], 1.0]))
        ET.SubElement(material, "ambient").text = color
        ET.SubElement(material, "diffuse").text = color
    sensor = ET.SubElement(link, "sensor", name="rgbd", type="depth")
    ET.SubElement(sensor, "always_on").text = "true"
    ET.SubElement(sensor, "update_rate").text = str(camera["update_rate"])
    lens = ET.SubElement(sensor, "camera")
    ET.SubElement(lens, "horizontal_fov").text = str(camera["horizontal_fov"])
    image = ET.SubElement(lens, "image")
    for key in ("width", "height"):
        ET.SubElement(image, key).text = str(camera[key])
    ET.SubElement(image, "format").text = "R8G8B8"
    clip = ET.SubElement(lens, "clip")
    for key in ("near", "far"):
        ET.SubElement(clip, key).text = str(camera[key])
    plugin = ET.SubElement(sensor, "plugin", name="shelf_rgbd", filename="libgazebo_ros_camera.so")
    ET.SubElement(ET.SubElement(plugin, "ros"), "namespace").text = "/perception_camera"
    ET.SubElement(plugin, "camera_name").text = "camera"
    ET.SubElement(plugin, "frame_name").text = "camera_optical_frame"
    # Match the pixel-center convention used by Gazebo's depth point cloud;
    # gazebo_ros_camera otherwise defaults to (width+1)/2, a one-pixel offset.
    ET.SubElement(plugin, "cx").text = str((camera["width"] - 1)/2)
    ET.SubElement(plugin, "cy").text = str((camera["height"] - 1)/2)
    ET.SubElement(plugin, "min_depth").text = str(camera["near"])
    ET.SubElement(plugin, "max_depth").text = str(camera["far"])
    return ET.tostring(root, encoding="unicode")


def locate_cuboid(rgb, depth, intrinsics, camera_to_base, config):
    """Return center, pixel mask and visible extents, or reject incomplete views.

    Both images must be aligned and depth must be optical Z in meters. The
    roof-mounted camera sees the top face; known height converts that measured
    surface into the object's center, even when its front face is hidden.
    """
    if rgb.ndim != 3 or rgb.shape[2] != 3 or depth.shape != rgb.shape[:2]:
        raise ValueError("RGB and depth images must be aligned and have matching dimensions.")
    k = np.asarray(intrinsics, dtype=float).reshape(3, 3)
    if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError("Invalid camera intrinsics.")
    mask = cv2.inRange(cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV),
                       np.array(config["hsv_lower"], dtype=np.uint8),
                       np.array(config["hsv_upper"], dtype=np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    if count <= 1:
        raise ValueError("Orange object not visible.")
    label = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    mask = labels == label
    # Rendering anti-aliasing mixes object RGB with background depth at edges.
    interior = cv2.erode(mask.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
    valid = interior & np.isfinite(depth) & (depth > 0.05) & (depth < 3.0)
    v, u = np.nonzero(valid)
    if len(u) < config["min_pixels"]:
        raise ValueError("Not enough valid depth pixels on the orange object.")
    z = depth[v, u]
    points = np.column_stack(((u - k[0, 2]) * z / k[0, 0],
                              (v - k[1, 2]) * z / k[1, 1], z))
    points = points @ camera_to_base[:3, :3].T + camera_to_base[:3, 3]
    top_z = np.percentile(points[:, 2], 75)
    top = points[np.abs(points[:, 2] - top_z) < 0.002]
    if len(top) < config["min_pixels"]:
        raise ValueError("Not enough top-face pixels on the orange object.")
    lower, upper = np.percentile(top, [0.5, 99.5], axis=0)
    extents = upper - lower
    size = np.asarray(config["object_size"])
    top_area = cv2.contourArea(cv2.convexHull(top[:, :2].astype(np.float32)))
    if (np.any(extents[:2] < size[:2] * config["extent_min_ratio"])
            or np.any(extents[:2] > size[:2] * config["extent_max_ratio"])
            or top_area < size[0] * size[1] * config["min_top_area_ratio"]):
        raise ValueError("Object view is occluded, rotated, or inconsistent with the known cuboid.")
    center = (lower + upper) / 2
    center[2] = np.median(top[:, 2]) - size[2]/2
    return center, mask, extents
