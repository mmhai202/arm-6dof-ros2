"""Contact-only Gazebo grasping, with MoveIt used for collision-aware motion."""

import argparse
import copy
import json
import math
import random
import sys
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from arm_control.trajectory import (
    ACTION_NAME, MotionStopError, cancel_trajectory, trajectory_goal, wait_for_future,
)
from arm_description import GRIPPER_JOINT_NAMES, JOINT_NAMES
from arm_description.model import RobotModel
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from gazebo_msgs.msg import ContactsState, LinkStates
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from .gazebo_scene import (
    ENVIRONMENT_ID, environment_matches, gazebo_models, payload_is_settled,
    spawned_payloads_are_settled,
)
from .scene_ownership import SceneOwnership
from .perception_input import (
    DetectionWindow, PERCEPTION_PARAMETERS, sample_spawn_center, validate_perception_config,
)
from .shelf_scene import OBJECT_ID, ShelfScene, scene_diff
from .shelf_task import ShelfTask
from .task_state import TaskCanceled


SCENE_SYNC_TIMEOUT = 5.0


def pose_transform(pose):
    """Convert a Gazebo world pose to a homogeneous transform."""
    p, q = pose.position, pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    transform = np.eye(4)
    transform[:3, :3] = [
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ]
    transform[:3, 3] = [p.x, p.y, p.z]
    return transform


class GazeboShelfTask(ShelfTask):
    def __init__(self, *, motion_overrides=None, use_global_arguments=True,
                 node_name="shelf_pick_place", perception_overrides=None, scene=None):
        if scene is None:
            scene = ShelfScene()
        config = copy.deepcopy(scene.config)
        config["motion"].update(motion_overrides or {})
        super().__init__(type(scene)(config), use_global_arguments=use_global_arguments,
                         node_name=node_name)
        self.scene.config["perception"].update(perception_overrides or {})
        for parameter, key in PERCEPTION_PARAMETERS.items():
            self.declare_parameter(parameter, self.scene.config["perception"][key])
            self.scene.config["perception"][key] = self.get_parameter(parameter).value
        self.use_perception = self.scene.config["perception"]["enabled"]
        validate_perception_config(self.scene.config["perception"])
        seed = self.scene.config["perception"]["spawn_seed"]
        self.spawn_rng = random.Random(None if seed < 0 else seed)
        self.spawn_center = list(self.scene.pick_center)
        self.observation = None
        self.detected_center = None
        self.camera_center = None
        self.grasp_verified = False
        self.pose_subscription = self.create_subscription(
            PoseStamped, self.scene.config["perception"]["pose_topic"],
            self.on_detection, 10)
        for parameter, key in (("payload_mass", "mass"), ("payload_friction", "friction"),
                               ("grip_compression", "grip_compression")):
            self.declare_parameter(parameter, self.scene.config["physics"][key])
            self.scene.config["physics"][key] = self.get_parameter(parameter).value
        self.models = gazebo_models(self.scene)
        self.physics = self.scene.config["physics"]
        self.payload_pose = None
        self.payload_twist = None
        self.model_stamp = None
        self.payload_center = None
        self.tool_tilt = 0.0
        self.world_links = ()
        self.world_states = None
        self.world_stamp = None
        self.link_generation = 0
        # Gazebo merges the fixed TCP/palm links into link_6. Derive that
        # constant offset from the URDF rather than duplicating tool dimensions.
        self.wrist_to_tcp = RobotModel.load(root_frame="link_6").tcp_transform([0.0] * len(JOINT_NAMES))
        self.last_contact = {side: -math.inf for side in ("left", "right")}
        self.contact_forces = {side: 0.0 for side in self.last_contact}
        self.holding = False
        self.grasp_center = None
        self.active_goal = None
        self.fault = None
        self.sync_enabled = False
        self.sync_future = None
        self.sync_started = None
        self.ownership = SceneOwnership()
        self.task_running = True
        self.stage = "initializing"
        self.report = self.create_publisher(String, "/shelf_grasp/status", 10)
        self.link_subscription = self.create_subscription(
            LinkStates, "/gazebo/link_states", self.on_links, qos_profile_sensor_data)
        self.contact_subscriptions = [self.create_subscription(
            ContactsState, f"/gripper/{side}_contacts",
            lambda message, side=side: self.on_contact(side, message), qos_profile_sensor_data,
        ) for side in self.last_contact]
        self.gripper = ActionClient(self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
        self.arm = ActionClient(self, FollowJointTrajectory, ACTION_NAME)
        self.sync_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.timer = self.create_timer(0.2, self.monitor)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_detection(self, message):
        if self.observation is not None:
            self.detected_center = self.observation.add(message, self.now())

    def observe_object(self):
        """Require fresh image measurements after reset, without any GT fallback."""
        self.report_progress("perception", "observe_object")
        self.observation = DetectionWindow(self.scene, after=self.now())
        self.detected_center = None
        timeout = self.scene.config["perception"]["timeout"]
        deadline = self.now() + timeout
        wall_deadline = time.monotonic() + max(30, timeout * 5)
        try:
            while self.now() < deadline and time.monotonic() < wall_deadline:
                self.check_cancel()
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.detected_center is not None:
                    center = self.detected_center
                    self.camera_center = list(center)
                    self.get_logger().info(f"Camera target in {self.scene.frame}: {center}")
                    return center
            raise RuntimeError(f"Perception timed out: {self.observation.reason}")
        finally:
            self.observation = None

    def on_links(self, message):
        self.world_links = tuple(message.name)
        self.world_states = message
        self.world_stamp = self.now()
        self.link_generation += 1
        object_id = getattr(self, "object_id", OBJECT_ID)
        wrist_name, payload_name = "arm_sim::link_6", f"{object_id}::{object_id}"
        if wrist_name not in message.name or payload_name not in message.name:
            return  # Keep the last timestamp so missing feedback expires.
        wrist = message.pose[message.name.index(wrist_name)]
        payload = message.pose[message.name.index(payload_name)]
        # Both poses come from the same physics update. Mixing model_states
        # with the latest joint_states makes sensor delay look like payload slip.
        tcp = pose_transform(wrist) @ self.wrist_to_tcp
        self.tool_tilt = math.acos(float(np.clip(-tcp[2, 2], -1.0, 1.0)))
        p = payload.position
        self.payload_center = tcp[:3, :3].T @ (np.array([p.x, p.y, p.z]) - tcp[:3, 3])
        self.payload_pose = payload
        index = message.name.index(payload_name)
        self.payload_twist = message.twist[index] if index < len(message.twist) else None
        self.model_stamp = self.now()

    def on_contact(self, side, message):
        samples = []
        for state in message.states:
            if any(name.split("::")[0] == getattr(self, "object_id", OBJECT_ID)
                   for name in (state.collision1_name, state.collision2_name)):
                f = state.total_wrench.force
                samples.append(math.sqrt(f.x*f.x + f.y*f.y + f.z*f.z))
        # The bumper batches several physics steps in one sensor message.
        # Averaging avoids multiplying the reported newtons by that batch size.
        force = sum(samples) / len(samples) if samples else 0.0
        self.contact_forces[side] = force
        if force >= self.physics["min_contact_force"]:
            self.last_contact[side] = self.now()

    def bilateral_contact(self):
        return all(self.now() - stamp < self.physics["contact_timeout"]
                   for stamp in self.last_contact.values())

    def measured_payload(self):
        if self.use_perception and not self.grasp_verified:
            # Before grasping, only the camera estimate may enter MoveIt.
            if self.camera_center is None:
                raise RuntimeError("No valid camera target is available for the planning scene.")
            return self.scene.payload(self.scene.pick_center)
        if self.payload_pose is None:
            raise RuntimeError("No Gazebo payload pose received.")
        obj = self.scene.payload([0, 0, 0])
        obj.pose = copy.deepcopy(self.payload_pose)
        return obj

    def local_payload_center(self):
        if self.payload_center is None or self.now() - self.model_stamp > self.physics["contact_timeout"]:
            raise RuntimeError("Missing or stale synchronized wrist/payload feedback.")
        return self.payload_center.copy()

    def check_sync_timeout(self):
        if self.sync_future is not None and time.monotonic() - self.sync_started > SCENE_SYNC_TIMEOUT:
            self.sync_enabled = False
            self.ownership.quarantine()
            raise TimeoutError("Planning-scene synchronization was not acknowledged within 5 seconds.")

    def monitor(self):
        if not self.sync_enabled or self.payload_pose is None:
            return
        if not self.ownership.resume():
            return
        self.check_sync_timeout()
        drift = None
        if self.holding and self.fault is None:
            if self.now() - self.model_stamp > self.physics["contact_timeout"]:
                self.fault = "Gazebo payload feedback stopped."
            elif not self.bilateral_contact():
                self.fault = "Lost bilateral finger contact; payload may have slipped."
            else:
                drift = np.linalg.norm(self.local_payload_center() - self.grasp_center)
                if drift > self.physics["max_grasp_drift"]:
                    self.fault = f"Payload slipped {drift:.3f} m relative to the TCP."
            if self.fault and self.active_goal is not None:
                self.active_goal.cancel_goal_async()
                self.hold_arm()
        if not self.holding and self.sync_future is None:
            # A dedicated asynchronous client avoids recursive spinning in timers.
            diff = scene_diff()
            diff.world.collision_objects = [self.measured_payload()]
            self.sync_started = time.monotonic()
            self.ownership.quarantine()
            self.sync_future = self.sync_client.call_async(ApplyPlanningScene.Request(scene=diff))
            self.sync_future.add_done_callback(self.on_sync_done)
        p = self.payload_pose.position
        self.report.publish(String(data=json.dumps({
            "stage": self.stage, "holding": self.holding, "fault": self.fault,
            "payload": [p.x, p.y, p.z], "finger_forces": self.contact_forces,
            "grasp_drift": None if drift is None else float(drift),
            "tool_tilt": self.tool_tilt if self.holding else None,
        })))
        if not self.task_running and self.sync_future is None:
            self.ownership.release()

    def on_sync_done(self, future):
        if self.sync_future is not future:
            return
        try:
            response = future.result()
            self.ownership.acknowledge()
            if not response.success:
                self.fault = "MoveIt rejected payload synchronization."
        except Exception as error:
            self.sync_enabled = False
            self.fault = f"Payload synchronization failed: {error}"
        finally:
            self.sync_future = None
            if not self.task_running:
                self.ownership.release()

    def acquire_scene(self, timeout=10):
        self.check_cancel()
        deadline = time.monotonic() + timeout
        while not self.ownership.claim():
            self.check_cancel()
            if time.monotonic() >= deadline:
                raise RuntimeError("Another shelf task is busy or awaiting a scene update; retry after it finishes.")
            rclpy.spin_once(self, timeout_sec=0.05)

    def stop_sync(self):
        self.sync_enabled = False
        future = self.sync_future
        if future is not None:
            # Keep ownership until MoveIt acknowledges the last write. A timeout
            # must not release the lock with an update still in flight.
            rclpy.spin_until_future_complete(self, future, timeout_sec=SCENE_SYNC_TIMEOUT)
            if not future.done():
                self.ownership.quarantine()
                raise RuntimeError("Last planning-scene update was not acknowledged within 5 seconds; "
                                   "scene ownership is quarantined until acknowledgement or recovery.")
            # The executor may have queued the done callback for a later spin.
            self.on_sync_done(future)

    def idle(self):
        self.stop_sync()
        self.task_running = False
        self.ownership.release()
        self.sync_enabled = not (self.use_perception and not self.grasp_verified and self.camera_center is None)

    def hold_arm(self):
        """Preempt the owned controller motion even if MoveIt ignores cancellation."""
        if self.latest_joints is None or not self.arm.server_is_ready():
            return
        positions = dict(zip(self.latest_joints.name, self.latest_joints.position))
        goal = trajectory_goal([([positions[name] for name in JOINT_NAMES], 0.2)])
        goal.trajectory.points[0].velocities = [0.0] * len(JOINT_NAMES)
        self.stop_future = self.arm.send_goal_async(goal)

    def advance(self, seconds):
        target = self.now() + seconds
        deadline = time.monotonic() + max(30, seconds * 5)
        while self.now() < target and time.monotonic() < deadline:
            self.check_cancel()
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.now() < target:
            raise RuntimeError("Gazebo simulation clock is not advancing.")
        if self.fault:
            raise RuntimeError(self.fault)

    def wait_for_placement(self):
        """Require fresh, consecutive physics samples of a stationary landed body."""
        timeout = self.physics["placement_timeout"]
        deadline = self.now() + timeout
        wall_deadline = time.monotonic() + max(30, timeout * 5)
        stable_since = last_stamp = None
        while self.now() < deadline and time.monotonic() < wall_deadline:
            self.check_cancel()
            if self.fault:
                raise RuntimeError(self.fault)
            stamp = self.model_stamp
            fresh = stamp is not None and 0 <= self.now() - stamp < self.physics["contact_timeout"]
            if not fresh:
                stable_since = None
            elif last_stamp is None or stamp > last_stamp:
                if last_stamp is not None and stamp - last_stamp >= self.physics["contact_timeout"]:
                    stable_since = None
                last_stamp = stamp
                if payload_is_settled(self.scene, self.payload_pose, self.payload_twist):
                    if stable_since is None:
                        stable_since = stamp
                    if stamp - stable_since >= self.physics["placement_hold_time"]:
                        return
                else:
                    stable_since = None
            rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError("Release failed: payload did not remain stationary inside the box "
                           "with fresh Gazebo feedback.")

    def wait_simulation_result(self, future, timeout):
        """Allow slow physics to finish while detecting a stopped simulation clock."""
        previous = self.now()
        deadline = previous + timeout
        last_progress = time.monotonic()
        while not future.done():
            self.check_cancel()
            current = self.now()
            if current > previous:
                previous = current
                last_progress = time.monotonic()
            if current >= deadline:
                raise TimeoutError("Trajectory exceeded its simulation-time deadline.")
            if time.monotonic() - last_progress > 30:
                raise TimeoutError("Gazebo simulation clock stopped while executing a trajectory.")
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.result()

    def wait_for_models(self, present, names=None):
        """Wait for a physics snapshot confirming the whole scene transition."""
        generation = self.link_generation
        expected = set(self.models if names is None else names)
        deadline = time.monotonic() + 10
        while True:
            self.check_cancel()
            found = {name for name in expected
                     if any(link.startswith(f"{name}::") for link in self.world_links)}
            ready = found == expected if present else not found
            if self.link_generation > generation and ready:
                return
            if time.monotonic() >= deadline:
                action = "creating" if present else "removing"
                raise RuntimeError(f"Gazebo did not confirm {action} the demo scene.")
            rclpy.spin_once(self, timeout_sec=0.05)

    def spawn_scene(self):
        # Confirm the scene itself, not just the factory's replies. Retry
        # setup only, before planning or motion.
        for attempt in range(3):
            try:
                self.reset_scene()
                return
            except RuntimeError as error:
                if attempt == 2:
                    raise
                self.get_logger().warning(f"Scene reset incomplete: {error}; retrying before motion.")

    def reset_scene(self):
        self.models = gazebo_models(self.scene)
        # A fresh physics snapshot also covers an empty initial scene. Some
        # Gazebo versions fail to reply to GetEntityState for missing models.
        self.wait_for_models(present=True, names=())
        keep_environment = environment_matches(
            self.models[ENVIRONMENT_ID], getattr(self, "world_states", None))
        reset_models = {name: xml for name, xml in self.models.items()
                        if name != ENVIRONMENT_ID or not keep_environment}
        existing = {name for name in reset_models
                    if any(link.startswith(f"{name}::") for link in self.world_links)}
        # Remove the payload before its supports, then finish all removals
        # before reusing model names. A service reply alone is not a scene snapshot.
        for name in reversed(reset_models):
            if name in existing:
                result = self.call(DeleteEntity, "/delete_entity", DeleteEntity.Request(name=name))
                if not result.success:
                    raise RuntimeError(f"Cannot reset {name}: {result.status_message}")
        if keep_environment:
            self.get_logger().info("Reusing unchanged Gazebo fixtures.")
            self.wait_for_models(present=False, names=tuple(reset_models))
        else:
            self.wait_for_models(present=False)
        self.payload_pose = self.payload_center = self.model_stamp = None
        self.detected_center = None
        self.camera_center = None
        self.grasp_verified = False
        self.payload_twist = None
        self.last_contact = dict.fromkeys(self.last_contact, -math.inf)
        self.contact_forces = dict.fromkeys(self.contact_forces, 0.0)
        for name, xml in reset_models.items():
            request = SpawnEntity.Request(name=name, xml=xml)
            request.initial_pose.orientation.w = 1.0
            spawn_positions = getattr(self.scene, "spawn_positions", {OBJECT_ID: self.spawn_center})
            if name in spawn_positions:
                p = request.initial_pose.position
                p.x, p.y, p.z = map(float, spawn_positions[name])
            response = self.call(SpawnEntity, "/spawn_entity", request)
            if not response.success:
                raise RuntimeError(f"Cannot spawn {name}: {response.status_message}")
            # The factory can reply before the model's links are initialized.
            # Finish that insertion before sending the next factory request.
            self.wait_for_models(present=True, names=(name,))
        found = {name for name in self.models
                 if any(link.startswith(f"{name}::") for link in self.world_links)}
        if found != set(self.models):
            raise RuntimeError("Gazebo removed a model while creating the demo scene.")
        self.wait_for_spawn_settle()
        if self.payload_pose is None or self.now() - self.model_stamp > self.physics["contact_timeout"]:
            raise RuntimeError("Gazebo did not publish fresh feedback for the spawned payload.")
        p = self.payload_pose.position
        if np.linalg.norm(np.array([p.x, p.y, p.z]) - self.spawn_center) > self.physics["max_grasp_drift"]:
            raise RuntimeError("Spawned payload did not settle at the shelf pick position.")

    def wait_for_spawn_settle(self):
        """Finish init as soon as fresh physics confirms every object is stable."""
        centers = getattr(self.scene, "spawn_positions", {OBJECT_ID: self.spawn_center})
        hold = self.physics["placement_hold_time"]
        timeout = max(self.physics["settle_time"], hold + self.physics["contact_timeout"])
        after = self.now()
        deadline = after + timeout
        wall_deadline = time.monotonic() + max(30, timeout * 5)
        stable_since = last_stamp = None
        while self.now() < deadline and time.monotonic() < wall_deadline:
            self.check_cancel()
            if self.fault:
                raise RuntimeError(self.fault)
            stamp = self.world_stamp
            fresh = stamp is not None and stamp > after and 0 <= self.now() - stamp < self.physics["contact_timeout"]
            if not fresh:
                stable_since = None
            elif last_stamp is None or stamp > last_stamp:
                if last_stamp is not None and stamp - last_stamp >= self.physics["contact_timeout"]:
                    stable_since = None
                last_stamp = stamp
                if spawned_payloads_are_settled(self.scene, self.world_states, centers):
                    if stable_since is None:
                        stable_since = stamp
                    if stamp - stable_since >= hold:
                        return
                else:
                    stable_since = None
            rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError("Spawned payloads did not settle near their reset positions with fresh Gazebo feedback.")

    def squeeze(self):
        if not self.gripper.wait_for_server(timeout_sec=20):
            raise RuntimeError("Gripper controller is unavailable.")
        command = self.scene.gripper_closed - self.physics["grip_compression"]
        goal = trajectory_goal(
            [([command] * len(GRIPPER_JOINT_NAMES), 2)], joint_names=GRIPPER_JOINT_NAMES,
        )
        goal.trajectory.points[0].velocities = [0.0] * len(GRIPPER_JOINT_NAMES)
        # The object blocks closure. Keep the spring target past contact without
        # treating that deliberate position error as a failed trajectory.
        goal.goal_tolerance = [JointTolerance(
            name=name, position=self.physics["grip_compression"] + 0.004, velocity=0.02,
        ) for name in GRIPPER_JOINT_NAMES]
        handle = wait_for_future(self.gripper.send_goal_async(goal), 15, node=self)
        if handle is None or not handle.accepted:
            raise RuntimeError("Gripper rejected the squeeze command.")
        try:
            self.check_cancel()
            result = self.wait_simulation_result(handle.get_result_async(), 15)
        except (TaskCanceled, TimeoutError, RuntimeError, KeyboardInterrupt):
            if rclpy.ok(context=self.context):
                self.hold_arm()
                if not cancel_trajectory(handle, node=self):
                    raise MotionStopError("Could not confirm gripper cancellation.")
            raise
        if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f"Gripper failed: {result.result.error_string}")
        self.advance(0.4)
        if not self.bilateral_contact():
            raise RuntimeError("Grasp rejected: both fingers must contact the payload with force.")

    def change_scene(self, name):
        if not self.sync_enabled:
            return super().change_scene(name)  # Virtual attachment during preflight only.
        if name == "attach":
            # Drain any world-object update before removing it for planning.
            self.stop_sync()
            if self.fault:
                raise RuntimeError(self.fault)
            if not self.bilateral_contact():
                raise RuntimeError("Cannot lift: bilateral payload contact is missing.")
            self.grasp_center = self.local_payload_center()
            self.grasp_verified = True
            diff = self.scene.attach()
            p = diff.robot_state.attached_collision_objects[0].object.primitive_poses[0].position
            p.x, p.y, p.z = map(float, self.grasp_center)
            # This attachment is collision metadata for MoveIt only. Gazebo still
            # has a free body with no joint, constraint, pose setter or teleport.
            self.holding = True
            self.apply(diff)
            self.sync_enabled = True
        elif name == "detach":
            self.holding = False
            diff = self.scene.detach([0, 0, 0])
            diff.world.collision_objects = [self.measured_payload()]
            self.apply(diff)
        else:
            super().change_scene(name)

    def execute(self, trajectory):
        if self.fault:
            raise RuntimeError(self.fault)
        self.check_cancel()
        # The previous controller goal already checked position and velocity
        # tolerances. MoveIt validates this start against the same configured
        # position bound; do not impose another, tighter settling threshold.
        if not self.executor_client.wait_for_server(timeout_sec=15):
            raise RuntimeError("MoveIt execution is unavailable.")
        handle = wait_for_future(self.executor_client.send_goal_async(
            ExecuteTrajectory.Goal(trajectory=trajectory)), 15, node=self)
        if handle is None or not handle.accepted:
            raise RuntimeError("MoveIt rejected execution.")
        self.active_goal = handle
        try:
            self.check_cancel()
            duration = trajectory.joint_trajectory.points[-1].time_from_start
            result = self.wait_simulation_result(handle.get_result_async(), duration.sec + duration.nanosec / 1e9 + 30)
            if self.fault:
                raise RuntimeError(self.fault)
            if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code.val != MoveItErrorCodes.SUCCESS:
                raise RuntimeError(f"MoveIt execution failed: {result.result.error_code.val}.")
        except (TaskCanceled, TimeoutError, RuntimeError, KeyboardInterrupt):
            if rclpy.ok(context=self.context):
                self.hold_arm()
                if not cancel_trajectory(handle, node=self):
                    raise MotionStopError("Could not confirm MoveIt cancellation; arm hold requested.")
            raise
        finally:
            self.active_goal = None

    def run(self, mode):
        self.check_cancel()
        self.report_progress("initializing", "reset_scene")
        if self.use_perception and self.scene.config["perception"]["randomize_spawn"]:
            self.spawn_center = sample_spawn_center(self.scene, self.spawn_rng)
        self.spawn_scene()
        if self.use_perception:
            self.scene.pick_center = self.observe_object()
        self.apply(self.scene.initial())
        self.set_finger_contact(False)
        self.wait_for_joints()
        if mode == "scene":
            self.sync_enabled = True
            self.report_progress("ready", "scene_ready")
            self.get_logger().info("Gazebo cabinet, dynamic payload and box ready; poses synchronized to MoveIt.")
            return
        planned = self.plan_task()
        self.get_logger().info("All physical pick-and-place stages planned successfully.")
        self.sync_enabled = True
        if mode == "plan":
            self.report_progress("planned", "planned")
            return
        if self.use_perception:
            # Planning can take seconds. Recheck before any physical motion.
            center = self.observe_object()
            if np.linalg.norm(np.array(center) - self.scene.pick_center) > 0.006:
                raise RuntimeError("Camera target moved after planning; run the task again.")
        for index, (name, trajectory) in enumerate(planned):
            self.check_cancel()
            self.report_progress("executing", name, index, len(planned))
            self.get_logger().info(f"Executing physical stage: {name}")
            if name == "close_gripper":
                self.squeeze()
            elif trajectory is None:
                self.change_scene(name)
            else:
                self.execute(trajectory)
            if name == "lift":
                self.advance(0.3)
                if self.payload_pose.position.z < self.scene.pick_center[2] + self.scene.motion["lift"] * 0.6:
                    raise RuntimeError("Lift failed: Gazebo payload did not leave the shelf.")
                self.get_logger().info("Verified physical lift with bilateral contact and free payload motion.")
            if name == "release_gripper":
                # Fresh stationary samples determine when release is complete.
                self.wait_for_placement()
        self.report_progress("succeeded", "success", len(planned), len(planned))
        self.monitor()
        self.get_logger().info("SUCCESS: contact-only grasp lifted the free Gazebo payload and released it inside the box.")

    def retain_after_failure(self, error):
        self.fault = str(error)
        self.stage = "failed"
        self.holding = False
        if self.use_perception and not self.grasp_verified and self.camera_center is None:
            self.sync_enabled = False
            return  # No nominal or simulator position may replace a missing detection.
        # Never issue another scene write while the previous one is uncertain.
        if self.sync_future is not None or self.ownership.pending:
            self.sync_enabled = False
            return
        self.sync_enabled = True
        if self.payload_pose is not None:
            diff = self.scene.detach([0, 0, 0])
            diff.world.collision_objects = [self.measured_payload()]
            self.apply(diff)
        self.monitor()


def main(argv=None, task_factory=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("scene", "plan", "execute"))
    parser.add_argument("--once", action="store_true", help="Exit after the task instead of maintaining scene synchronization.")
    args, ros_args = parser.parse_known_args(argv)
    # SIGINT must reach Python without tearing down ROS before cancellation.
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = (task_factory or GazeboShelfTask)()
    result = 0
    try:
        # Rejected contenders must never reset models or retain/modify the scene.
        node.acquire_scene()
        try:
            node.run(args.mode or node.get_parameter("mode").value)
        except (RuntimeError, TimeoutError, ValueError) as error:
            if not rclpy.ok(context=node.context):
                return 0
            node.get_logger().error(f"Physical task stopped: {error}. Scene retained for inspection.")
            node.retain_after_failure(error)
            result = 1
        if not args.once:
            node.idle()
            while rclpy.ok(context=node.context) and not node.ownership.superseded:
                rclpy.spin_once(node, timeout_sec=0.2)
                # The ROS timer stops with /clock; enforce the service deadline
                # from this wall-clock loop as well.
                node.check_sync_timeout()
                if node.fault and node.ownership.pending and node.sync_future is None:
                    raise RuntimeError(node.fault)
    except (RuntimeError, TimeoutError, ValueError) as error:
        node.get_logger().error(str(error))
        result = 1
    except (KeyboardInterrupt, ExternalShutdownException):
        result = 130
    finally:
        try:
            if rclpy.ok(context=node.context):
                try:
                    node.stop_sync()
                except (RuntimeError, TimeoutError) as error:
                    node.get_logger().error(str(error))
                    result = 1
        finally:
            node.destroy_node()
            node.ownership.close()
            rclpy.try_shutdown()
    return result


if __name__ == "__main__":
    sys.exit(main())
