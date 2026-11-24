"""Image-derived multi-object poses for the factory picking deck."""

import json

import cv2
from cv_bridge import CvBridgeError
import numpy as np
import rclpy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import TransformException

from .detector import RGBDDetector
from .geometry import locate_cuboid, transform_matrix


def locate_stock(rgb, depth, intrinsics, camera_to_base, config):
    """Fit each connected component separately, including multiple same-color parts."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    result = []
    for color, threshold in config["colors"].items():
        mask = cv2.inRange(hsv, np.array(threshold["hsv_lower"], np.uint8),
                           np.array(threshold["hsv_upper"], np.uint8))
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        for label in range(1, count):
            if stats[label, cv2.CC_STAT_AREA] < config["min_pixels"]:
                continue
            isolated = rgb.copy()
            isolated[labels != label] = 0
            try:
                center, pixels, _ = locate_cuboid(isolated, depth, intrinsics, camera_to_base,
                                                  {**config, **threshold})
            except ValueError:
                continue
            if np.any(center < config["workspace_min"]) or np.any(center > config["workspace_max"]):
                continue
            v, u = np.nonzero(pixels)
            result.append({"color": color, "center": center.tolist(),
                           "bounds": [int(u.min()), int(v.min()), int(u.max()), int(v.max())]})
    return result


class WarehouseDetector(RGBDDetector):
    def __init__(self):
        super().__init__()
        self.stock_pub = self.create_publisher(String, "/warehouse/detections", 10)

    def on_images(self, rgb_msg, depth_msg):
        debug = None
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
            for item in objects:
                x1, y1, x2, y2 = item["bounds"]
                cv2.rectangle(debug, (x1, y1), (x2, y2), (240, 255, 255), 1)
            self.status_pub.publish(String(data=json.dumps({"detected": bool(objects), "count": len(objects)})))
        except (ValueError, TransformException, cv2.error, CvBridgeError) as error:
            self.status_pub.publish(String(data=json.dumps({"detected": False, "reason": str(error)})))
        if debug is not None:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding="rgb8"))


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
