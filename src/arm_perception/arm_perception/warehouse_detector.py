"""Image-derived multi-object poses for the factory picking deck."""

from collections import Counter
import json

import cv2
from cv_bridge import CvBridgeError
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import TransformException

from .detector import RGBDDetector
from .geometry import locate_cuboid, transform_matrix


def draw_stock_overlay(image, objects, frame, *, unavailable=False):
    """Annotate a preview copy; labels identify detections in this frame only."""
    height, width = image.shape[:2]
    scale = width / 320
    font = cv2.FONT_HERSHEY_SIMPLEX
    background = (18, 24, 30)
    foreground = (240, 245, 250)
    colors = {"red": (255, 115, 115), "blue": (115, 190, 255),
              "green": (115, 245, 155)}

    def text(value, x, y, color=foreground):
        cv2.putText(image, value, (int(x), int(y)), font, 0.32 * scale,
                    color, max(1, round(scale)), cv2.LINE_AA)

    cv2.rectangle(image, (0, 0), (width - 1, round(18 * scale)), background, -1)
    counts = Counter(item["color"] for item in objects)
    if unavailable:
        text("DETECTION UNAVAILABLE", 5 * scale, 12 * scale, (255, 200, 100))
        return
    text(f"DETECTED {len(objects)} | RED {counts['red']}  BLUE {counts['blue']}  GREEN {counts['green']}",
         5 * scale, 12 * scale)
    if not objects:
        return

    row_height = 12 * scale
    rows = (len(objects) + 1) // 2
    footer = max(19 * scale, height - (rows + 1) * row_height - 4 * scale)
    cv2.rectangle(image, (0, int(footer)), (width - 1, height - 1), background, -1)
    text(f"XYZ (mm) / {frame}", 5 * scale, footer + 10 * scale)
    seen = Counter()
    for index, item in enumerate(objects):
        color = colors.get(item["color"], foreground)
        seen[item["color"]] += 1
        label = f"{item['color'][0].upper()}{seen[item['color']]}"
        x1, y1, x2, y2 = item["bounds"]
        cv2.rectangle(image, (x1, y1), (x2, y2), color, max(1, round(scale)))
        label_width = cv2.getTextSize(label, font, 0.32 * scale, 1)[0][0]
        x = max(0, min(x1, width - label_width - round(4 * scale)))
        y = max(round(29 * scale), y1 - round(3 * scale))
        cv2.rectangle(image, (x, y - round(10 * scale)),
                      (x + label_width + round(3 * scale), y + round(2 * scale)), background, -1)
        text(label, x + scale, y, color)
        xyz = " ".join(str(round(value * 1000)) for value in item["center"])
        text(f"{label}  {xyz}", (5 + (index % 2) * 160) * scale,
             footer + (22 + (index // 2) * 12) * scale, color)


def locate_stock(rgb, depth, intrinsics, camera_to_base, config):
    """Fit each connected component separately, including multiple same-color parts."""
    if rgb.ndim != 3 or rgb.shape[2] != 3 or depth.shape != rgb.shape[:2]:
        raise ValueError("RGB and depth images must be aligned and have matching dimensions.")
    k = np.asarray(intrinsics, dtype=float).reshape(3, 3)
    if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError("Invalid camera intrinsics.")
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    result = []
    for color, threshold in config["colors"].items():
        mask = cv2.inRange(hsv, np.array(threshold["hsv_lower"], np.uint8),
                           np.array(threshold["hsv_upper"], np.uint8))
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        for label in range(1, count):
            if stats[label, cv2.CC_STAT_AREA] < config["min_pixels"]:
                continue
            x, y, width, height = stats[label, :4]
            # Keep a background pixel around the component so erosion treats
            # its boundary exactly as it does in the complete image.
            x0, y0 = max(0, x - 1), max(0, y - 1)
            x1, y1 = min(rgb.shape[1], x + width + 1), min(rgb.shape[0], y + height + 1)
            isolated = rgb[y0:y1, x0:x1].copy()
            isolated[labels[y0:y1, x0:x1] != label] = 0
            local_k = k.copy()
            local_k[0, 2] -= x0
            local_k[1, 2] -= y0
            try:
                center, pixels, _ = locate_cuboid(
                    isolated, depth[y0:y1, x0:x1], local_k, camera_to_base,
                    {**config, **threshold})
            except ValueError:
                continue
            if np.any(center < config["workspace_min"]) or np.any(center > config["workspace_max"]):
                continue
            v, u = np.nonzero(pixels)
            result.append({"color": color, "center": center.tolist(),
                           "bounds": [int(u.min() + x0), int(v.min() + y0),
                                      int(u.max() + x0), int(v.max() + y0)]})
    return result


class WarehouseDetector(RGBDDetector):
    def __init__(self):
        super().__init__()
        # Small frame operations should not compete with physics for CPU workers.
        cv2.setNumThreads(1)
        self.stock_pub = self.create_publisher(String, "/warehouse/detections", 10)
        self.preview_image = None
        # Keep only the newest frame and cap the preview at 10 Hz of wall time.
        self.preview_timer = self.create_timer(
            0.1, self.publish_preview, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def publish_preview(self):
        if self.preview_image is not None:
            self.image_pub.publish(self.preview_image)
            self.preview_image = None

    def on_images(self, rgb_msg, depth_msg):
        debug = None
        objects = []
        unavailable = False
        try:
            if self.info is None:
                raise ValueError("Waiting for CameraInfo.")
            if (rgb_msg.header.frame_id != self.info.header.frame_id
                    or depth_msg.header.frame_id != rgb_msg.header.frame_id
                    or (rgb_msg.width, rgb_msg.height) != (self.info.width, self.info.height)):
                raise ValueError("RGB-D calibration does not match.")
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
            debug = rgb.copy()
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            if depth_msg.encoding == "16UC1":
                depth = depth.astype(np.float32) * 0.001
            elif depth_msg.encoding != "32FC1":
                raise ValueError("Unsupported depth encoding.")
            transform = self.tf_buffer.lookup_transform(
                self.config["frame"], rgb_msg.header.frame_id, Time.from_msg(rgb_msg.header.stamp))
            t, q = transform.transform.translation, transform.transform.rotation
            matrix = transform_matrix([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
            objects = locate_stock(rgb, depth, self.info.k, matrix, self.config["detection"])
            stamp = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec / 1e9
            self.stock_pub.publish(String(data=json.dumps({
                "frame": self.config["frame"], "stamp": stamp, "objects": objects})))
            self.status_pub.publish(String(data=json.dumps({"detected": bool(objects), "count": len(objects)})))
        except (ValueError, TransformException, cv2.error, CvBridgeError) as error:
            unavailable = True
            self.status_pub.publish(String(data=json.dumps({"detected": False, "reason": str(error)})))
        if debug is not None:
            draw_stock_overlay(debug, objects, self.config["frame"], unavailable=unavailable)
            image = self.bridge.cv2_to_imgmsg(debug, encoding="rgb8")
            image.header = rgb_msg.header
            self.preview_image = image


def main():
    rclpy.init()
    node = WarehouseDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
