"""Physics checks against a running, disposable Gazebo session.

ARM_RUN_GAZEBO_TESTS=1 python -m unittest discover -s tests -p test_gazebo_integration.py -v
Use the same ROS_DOMAIN_ID as run_moveit.sh.
These tests move the arm and temporarily spawn a falling cube.
"""

import math
import os
import time
import unittest
import uuid


ENABLED = os.environ.get("ARM_RUN_GAZEBO_TESTS") == "1"
if ENABLED:
    from arm_control.poses import load_named_poses
    from arm_control.trajectory import trajectory_goal
    from arm_description import JOINT_NAMES
    from control_msgs.action import FollowJointTrajectory
    from controller_manager_msgs.srv import ListControllers
    from gazebo_msgs.srv import DeleteEntity, GetEntityState, SpawnEntity
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState


@unittest.skipUnless(ENABLED, "Set ARM_RUN_GAZEBO_TESTS=1 with Gazebo running.")
class GazeboPhysicsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("gazebo_physics_regression")
        cls.stamp = None
        cls.joints = None
        cls.clock_subscription = cls.node.create_subscription(
            Clock, "/clock", lambda msg: setattr(cls, "stamp", msg.clock.sec + msg.clock.nanosec / 1e9),
            qos_profile_sensor_data,
        )
        cls.joint_subscription = cls.node.create_subscription(
            JointState, "/joint_states", lambda msg: setattr(cls, "joints", msg), qos_profile_sensor_data,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.try_shutdown()

    def wait_for(self, predicate, timeout=30):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(predicate(), "Timed out waiting for simulation progress/result")

    def advance(self, seconds):
        self.wait_for(lambda: self.stamp is not None)
        target = self.stamp + seconds
        self.wait_for(lambda: self.stamp >= target)

    def call(self, kind, name, request):
        client = self.node.create_client(kind, name)
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=20), name)
            future = client.call_async(request)
            self.wait_for(future.done)
            return future.result()
        finally:
            self.node.destroy_client(client)

    def entity(self, name):
        response = self.call(GetEntityState, "/gazebo/get_entity_state", GetEntityState.Request(name=name))
        self.assertTrue(response.success, name)
        return response.state

    def test_effort_controllers_track_and_hold_against_gravity(self):
        controllers = self.call(ListControllers, "/controller_manager/list_controllers", ListControllers.Request())
        by_name = {item.name: item for item in controllers.controller}
        for name in ("arm_controller", "gripper_controller"):
            self.assertEqual(by_name[name].state, "active")
            self.assertTrue(by_name[name].claimed_interfaces)
            self.assertTrue(all(item.endswith("/effort") for item in by_name[name].claimed_interfaces))
        base = self.entity("arm_sim::base_footprint").pose
        client = ActionClient(self.node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.assertTrue(client.wait_for_server(timeout_sec=20))
        try:
            for pose in ("ready", "home"):
                target = load_named_poses()[pose]
                future = client.send_goal_async(trajectory_goal([(target, 4)]))
                self.wait_for(future.done)
                handle = future.result()
                self.assertTrue(handle.accepted)
                result = handle.get_result_async()
                self.wait_for(result.done)
                self.assertEqual(result.result().result.error_code, 0, result.result().result.error_string)
                self.advance(3)
                self.assertIsNotNone(self.joints)
                positions = dict(zip(self.joints.name, self.joints.position))
                for joint, expected in zip(JOINT_NAMES, target):
                    self.assertAlmostEqual(positions[joint], expected, delta=0.02, msg=joint)
                self.assertTrue(all(math.isfinite(value) for value in self.joints.effort))
                self.assertGreater(max(abs(value) for value in self.joints.effort), 0.1)
        finally:
            client.destroy()
        after = self.entity("arm_sim::base_footprint").pose
        for axis in "xyz":
            self.assertAlmostEqual(getattr(base.position, axis), getattr(after.position, axis), delta=1e-5)

    def test_gravity_and_floor_contact(self):
        name = "physics_test_" + uuid.uuid4().hex
        request = SpawnEntity.Request(name=name, xml='''<sdf version="1.6"><model name="cube">
          <link name="body"><inertial><mass>0.1</mass><inertia>
          <ixx>0.00006</ixx><iyy>0.00006</iyy><izz>0.00006</izz>
          </inertia></inertial><collision name="collision"><geometry><box><size>0.06 0.06 0.06</size>
          </box></geometry></collision></link></model></sdf>''')
        request.initial_pose.position.x = 2.0
        request.initial_pose.position.y = 2.0
        request.initial_pose.position.z = 1.0
        request.initial_pose.orientation.w = 1.0
        response = self.call(SpawnEntity, "/spawn_entity", request)
        self.assertTrue(response.success, response.status_message)
        try:
            start = self.entity(name).pose.position.z
            self.advance(0.25)
            falling = self.entity(name)
            self.assertLess(falling.pose.position.z, start - 0.15)
            self.assertLess(falling.twist.linear.z, -1.0)
            self.advance(2.0)
            landed = self.entity(name)
            self.assertAlmostEqual(landed.pose.position.z, 0.03, delta=0.005)
            self.assertLess(abs(landed.twist.linear.z), 0.05)
        finally:
            self.call(DeleteEntity, "/delete_entity", DeleteEntity.Request(name=name))

    def test_moveit_plans_and_executes_on_gazebo(self):
        client = ActionClient(self.node, MoveGroup, "/move_action")
        self.assertTrue(client.wait_for_server(timeout_sec=20))
        try:
            goal = MoveGroup.Goal()
            request = goal.request
            request.group_name = "arm"
            request.allowed_planning_time = 5.0
            request.num_planning_attempts = 3
            request.max_velocity_scaling_factor = 0.2
            request.max_acceleration_scaling_factor = 0.2
            request.start_state.is_diff = True
            request.goal_constraints = [Constraints(joint_constraints=[
                JointConstraint(joint_name=name, position=value,
                                tolerance_above=0.001, tolerance_below=0.001, weight=1.0)
                for name, value in zip(JOINT_NAMES, load_named_poses()["ready"])
            ])]
            future = client.send_goal_async(goal)
            self.wait_for(future.done)
            handle = future.result()
            self.assertTrue(handle.accepted)
            result = handle.get_result_async()
            self.wait_for(result.done)
            self.assertEqual(result.result().result.error_code.val, MoveItErrorCodes.SUCCESS)
            self.assertTrue(result.result().result.planned_trajectory.joint_trajectory.points)
            self.advance(2.0)
            actual = dict(zip(self.joints.name, self.joints.position))
            for name, target in zip(JOINT_NAMES, load_named_poses()["ready"]):
                self.assertAlmostEqual(actual[name], target, delta=0.02, msg=name)
        finally:
            client.destroy()
