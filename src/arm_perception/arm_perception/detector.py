"""Spawn a fixed RGB-D sensor and publish stamped, image-derived object poses."""

import json
import math

import cv2
from cv_bridge import CvBridge, CvBridgeError
from arm_description.resources import load_yaml
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import PoseStamped, TransformStamped
import message_filters
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import camera_sdf, locate_cuboid, quaternion_from_rpy, transform_matrix
from .visualization import camera_markers


class RGBDDetector(Node):
    def __init__(self):
        super().__init__("shelf_rgbd_detector")
        self.declare_parameter("camera_config", "")
        path = self.get_parameter("camera_config").value
        if path:
            import yaml
            with open(path, encoding="utf-8") as stream:
                self.config = yaml.safe_load(stream)
        else:
            self.config = load_yaml("arm_perception", "config/camera.yaml")
        self.bridge = CvBridge()
        self.info = None
        self.spawn_future = None
        self.pose_pub = self.create_publisher(PoseStamped, "/perception/object_pose", 10)
        self.image_pub = self.create_publisher(Image, "/perception/debug_image", qos_profile_sensor_data)
        self.marker_pub = self.create_publisher(Marker, "/perception/object_marker", 10)
        self.camera_marker_pub = self.create_publisher(
            MarkerArray, "/perception/camera_markers",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.camera_visuals = None
        # Repaint after RViz Reset without respawning the sensor or waiting
        # for the simulation clock to advance.
        self.camera_marker_timer = self.create_timer(
            1.0, self.publish_camera_markers, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.status_pub = self.create_publisher(String, "/perception/status", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
        self.spawn_timer = self.create_timer(0.5, self.spawn_camera)
        self.info_sub = self.create_subscription(
            CameraInfo, "/perception_camera/camera/camera_info", self.on_info, qos_profile_sensor_data)
        self.rgb_sub = message_filters.Subscriber(
            self, Image, "/perception_camera/camera/image_raw", qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(
            self, Image, "/perception_camera/camera/depth/image_raw", qos_profile=qos_profile_sensor_data)
        # Gazebo RGB and depth have identical timestamps. Exact synchronization
        # prevents old RGB masks from being applied to a different depth frame.
        self.sync = message_filters.TimeSynchronizer([self.rgb_sub, self.depth_sub], 5)
        self.sync.registerCallback(self.on_images)

    def on_info(self, message):
        self.info = message

    def publish_camera_markers(self):
        if self.camera_visuals is not None:
            self.camera_marker_pub.publish(self.camera_visuals)

    def spawn_camera(self):
        if self.spawn_future is not None or not self.spawn_client.service_is_ready():
            return
        request = SpawnEntity.Request(name="perception_camera", xml=camera_sdf(self.config))
        camera = self.config["camera"]
        p, q = request.initial_pose.position, request.initial_pose.orientation
        p.x, p.y, p.z = map(float, camera["position"])
        q.x, q.y, q.z, q.w = quaternion_from_rpy(*camera["rpy"])
        self.spawn_future = self.spawn_client.call_async(request)
        self.spawn_future.add_done_callback(self.camera_ready)

    def camera_ready(self, future):
        self.spawn_timer.cancel()
        try:
            result = future.result()
            if not result.success:
                raise RuntimeError(result.status_message)
            camera = self.config["camera"]
            transforms = []
            for parent, child, xyz, rpy in (
                (self.config["frame"], "camera_link", camera["position"], camera["rpy"]),
                ("camera_link", "camera_optical_frame", [0.0]*3, [-math.pi/2, 0.0, -math.pi/2]),
            ):
                transform = TransformStamped()
                transform.header.frame_id = parent
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.child_frame_id = child
                t, q = transform.transform.translation, transform.transform.rotation
                t.x, t.y, t.z = map(float, xyz)
                q.x, q.y, q.z, q.w = quaternion_from_rpy(*rpy)
                transforms.append(transform)
            self.static_tf.sendTransform(transforms)
            self.camera_visuals = camera_markers(self.config)
            self.publish_camera_markers()
            self.get_logger().info("RGB-D camera ready; waiting for an unobstructed orange cuboid.")
        except Exception as error:
            self.get_logger().error(f"Cannot create calibrated RGB-D camera: {error}")
            self.status_pub.publish(String(data=json.dumps({"detected": False, "reason": str(error)})))

    def on_images(self, rgb_msg, depth_msg):
        debug = None
        try:
            if self.info is None:
                raise ValueError("Waiting for CameraInfo.")
            if (rgb_msg.header.frame_id != self.info.header.frame_id
                    or depth_msg.header.frame_id != rgb_msg.header.frame_id
                    or (rgb_msg.width, rgb_msg.height) != (self.info.width, self.info.height)):
                raise ValueError("CameraInfo, RGB and depth calibration do not match.")
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
            debug = rgb.copy()
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            if depth_msg.encoding == "16UC1":
                depth = depth.astype(np.float32) * 0.001
            elif depth_msg.encoding != "32FC1":
                raise ValueError(f"Unsupported depth encoding: {depth_msg.encoding}")
            transform = self.tf_buffer.lookup_transform(
                self.config["frame"], rgb_msg.header.frame_id, Time.from_msg(rgb_msg.header.stamp))
            t, q = transform.transform.translation, transform.transform.rotation
            matrix = transform_matrix([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
            center, mask, extents = locate_cuboid(rgb, depth, self.info.k, matrix, self.config["detection"])
            message = PoseStamped()
            message.header.frame_id = self.config["frame"]
            message.header.stamp = rgb_msg.header.stamp
            message.pose.position.x, message.pose.position.y, message.pose.position.z = map(float, center)
            message.pose.orientation.w = 1.0
            self.pose_pub.publish(message)
            marker = Marker(header=message.header, ns="perception", id=0, type=Marker.CUBE,
                            action=Marker.ADD, pose=message.pose)
            marker.scale.x, marker.scale.y, marker.scale.z = map(float, self.config["detection"]["object_size"])
            marker.color.r, marker.color.g, marker.color.a = 0.1, 1.0, 0.6
            marker.lifetime.sec = 1
            self.marker_pub.publish(marker)
            v, u = np.nonzero(mask)
            cv2.rectangle(debug, (int(u.min()), int(v.min())), (int(u.max()), int(v.max())), (0, 255, 0), 1)
            cv2.putText(debug, "XYZ " + " ".join(f"{v:.3f}" for v in center),
                        (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
            self.status_pub.publish(String(data=json.dumps({
                "detected": True, "center": center.tolist(), "extents": extents.tolist()})))
        except (ValueError, TransformException, cv2.error, CvBridgeError) as error:
            self.status_pub.publish(String(data=json.dumps({"detected": False, "reason": str(error)})))
            if debug is not None:
                cv2.putText(debug, "Waiting for clear object view", (5, 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 80, 0), 1)
        if debug is not None:
            image = self.bridge.cv2_to_imgmsg(debug, encoding="rgb8")
            image.header = rgb_msg.header
            self.image_pub.publish(image)


def main():
    rclpy.init()
    node = RGBDDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
