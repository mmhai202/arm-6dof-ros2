"""Read-only checks against running MoveIt: ARM_RUN_MOVEIT_TESTS=1 python -m unittest
 discover -s tests -p test_moveit_integration.py -v (activate the workspace first).
"""

import copy
import math
import os
import unittest


ENABLED = os.environ.get("ARM_RUN_MOVEIT_TESTS") == "1"
if ENABLED:
    from arm_control.poses import load_named_poses
    from arm_description import JOINT_NAMES
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import (
        BoundingVolume, Constraints, MoveItErrorCodes, OrientationConstraint,
        PositionConstraint, RobotState,
    )
    from moveit_msgs.srv import GetMotionPlan, GetPositionFK, GetPositionIK
    import rclpy
    from sensor_msgs.msg import JointState
    from shape_msgs.msg import SolidPrimitive


def rotate(pose, axis, angle):
    delta = [0.0, 0.0, 0.0, math.cos(angle / 2)]
    delta[axis] = math.sin(angle / 2)
    x, y, z, w = delta
    q = pose.pose.orientation
    values = (
        w * q.x + x * q.w + y * q.z - z * q.y,
        w * q.y - x * q.z + y * q.w + z * q.x,
        w * q.z + x * q.y - y * q.x + z * q.w,
        w * q.w - x * q.x - y * q.y - z * q.z,
    )
    for name, value in zip("xyzw", values):
        setattr(q, name, value)


@unittest.skipUnless(ENABLED, "Set ARM_RUN_MOVEIT_TESTS=1 with MoveIt running.")
class MoveItGoalTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("moveit_goal_regression")
        cls.clients = {}
        cls.ready = RobotState(joint_state=JointState(
            name=list(JOINT_NAMES), position=list(load_named_poses()["ready"]),
        ))
        request = GetPositionFK.Request(robot_state=cls.ready, fk_link_names=["tcp"])
        request.header.frame_id = "base_footprint"
        result = cls.call(GetPositionFK, "/compute_fk", request)
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError("Cannot compute the ready TCP pose.")
        cls.pose = result.pose_stamped[0]

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.try_shutdown()

    @classmethod
    def call(cls, service_type, name, request):
        if name not in cls.clients:
            cls.clients[name] = cls.node.create_client(service_type, name)
            if not cls.clients[name].wait_for_service(timeout_sec=10):
                raise RuntimeError(f"MoveIt service is unavailable: {name}")
        future = cls.clients[name].call_async(request)
        rclpy.spin_until_future_complete(cls.node, future, timeout_sec=15)
        if not future.done():
            raise TimeoutError(f"MoveIt service timed out: {name}")
        return future.result()

    def inverse_kinematics(self, pose):
        request = GetPositionIK.Request()
        request.ik_request.group_name = "arm"
        request.ik_request.ik_link_name = "tcp"
        request.ik_request.robot_state = self.ready
        request.ik_request.pose_stamped = pose
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.nanosec = 200_000_000
        return self.call(GetPositionIK, "/compute_ik", request)

    def test_ready_allows_small_translation_and_rotation(self):
        for kind in ("translation", "rotation"):
            for index, axis in enumerate("xyz"):
                for sign in (-1, 1):
                    with self.subTest(kind=kind, axis=axis, sign=sign):
                        pose = copy.deepcopy(self.pose)
                        if kind == "translation":
                            setattr(pose.pose.position, axis,
                                    getattr(pose.pose.position, axis) + sign * 0.05)
                        else:
                            rotate(pose, index, sign * 0.2)
                        result = self.inverse_kinematics(pose)
                        self.assertEqual(result.error_code.val, MoveItErrorCodes.SUCCESS)

    def test_translated_and_rotated_pose_goals_can_be_planned(self):
        for kind in ("translation", "rotation"):
            with self.subTest(kind=kind):
                pose = copy.deepcopy(self.pose)
                if kind == "translation":
                    pose.pose.position.x += 0.05
                else:
                    rotate(pose, 1, 0.2)
                region = Pose(position=copy.deepcopy(pose.pose.position))
                region.orientation.w = 1.0
                position = PositionConstraint(
                    header=pose.header, link_name="tcp", weight=1.0,
                    constraint_region=BoundingVolume(
                        primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.003])],
                        primitive_poses=[region],
                    ),
                )
                orientation = OrientationConstraint(
                    header=pose.header, link_name="tcp", orientation=pose.pose.orientation,
                    absolute_x_axis_tolerance=0.01, absolute_y_axis_tolerance=0.01,
                    absolute_z_axis_tolerance=0.01, weight=1.0,
                )
                request = GetMotionPlan.Request()
                plan = request.motion_plan_request
                plan.group_name = "arm"
                plan.start_state = self.ready
                plan.allowed_planning_time = 5.0
                plan.num_planning_attempts = 3
                plan.goal_constraints = [Constraints(
                    position_constraints=[position], orientation_constraints=[orientation],
                )]
                response = self.call(GetMotionPlan, "/plan_kinematic_path", request)
                self.assertEqual(response.motion_plan_response.error_code.val, MoveItErrorCodes.SUCCESS)
                self.assertTrue(response.motion_plan_response.trajectory.joint_trajectory.points)

    def test_out_of_reach_goal_is_rejected(self):
        pose = copy.deepcopy(self.pose)
        pose.pose.position.z += 10.0
        response = self.inverse_kinematics(pose)
        self.assertEqual(response.error_code.val, MoveItErrorCodes.NO_IK_SOLUTION)
