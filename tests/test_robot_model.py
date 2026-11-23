"""URDF forward kinematics regression; activate Conda and source the workspace."""

import unittest
import xml.etree.ElementTree as ET

try:
    from arm_control.poses import load_named_poses
    from arm_description.model import RobotModel
    from arm_description.resources import package_file
    from arm_description import JOINT_NAMES
    import xacro
    import numpy as np
except ImportError as exc:
    raise unittest.SkipTest("Activate Conda and source install/setup.bash") from exc


class RobotModelTest(unittest.TestCase):
    def test_physics_inertias_are_valid(self):
        root = ET.fromstring(xacro.process_file(str(package_file(
            "arm_description", "urdf/arm.urdf.xacro"))).toxml())
        for link in root.findall("link"):
            if link.find("collision") is None:
                continue
            with self.subTest(link=link.get("name")):
                inertia = link.find("inertial")
                self.assertIsNotNone(inertia)
                self.assertGreater(float(inertia.find("mass").get("value")), 0)
                values = {key: float(value) for key, value in inertia.find("inertia").attrib.items()}
                matrix = np.array([[values["ixx"], values["ixy"], values["ixz"]],
                                   [values["ixy"], values["iyy"], values["iyz"]],
                                   [values["ixz"], values["iyz"], values["izz"]]])
                moments = np.linalg.eigvalsh(matrix)
                self.assertTrue(np.all(moments > 0))
                self.assertLessEqual(moments[-1], moments[0] + moments[1] + 1e-10)

    def test_robot_uses_only_gazebo_effort_hardware(self):
        root = ET.fromstring(xacro.process_file(str(package_file(
            "arm_description", "urdf/arm.urdf.xacro"))).toxml())
        self.assertEqual(root.findtext("ros2_control/hardware/plugin"), "gazebo_ros2_control/GazeboSystem")
        self.assertIsNotNone(root.find("joint[@name='world_to_base']"))
        for joint in root.findall("ros2_control/joint"):
            self.assertEqual([item.get("name") for item in joint.findall("command_interface")], ["effort"])
        self.assertIsNotNone(root.find("gazebo/plugin[@name='gazebo_ros2_control']"))

    def test_hardware_starts_at_home(self):
        root = ET.fromstring(xacro.process_file(str(package_file(
            "arm_description", "urdf/arm.urdf.xacro"))).toxml())
        home = load_named_poses()["home"]
        for name, expected in zip(JOINT_NAMES, home):
            value = root.find(
                f"ros2_control/joint[@name='{name}']/"
                "state_interface[@name='position']/param[@name='initial_value']")
            self.assertIsNotNone(value, name)
            self.assertAlmostEqual(float(value.text), expected, places=8)

    def test_known_forward_kinematics(self):
        model = RobotModel.load()
        for values, expected in (
            ([0.0] * 6, (-0.8172, -0.3109, 0.0628)),
            (load_named_poses()["home"], (0.0, -0.3109, 1.0794)),
        ):
            tcp = model.tcp_position(values)
            for actual, reference in zip((tcp.x, tcp.y, tcp.z), expected):
                self.assertAlmostEqual(actual, reference, places=8)
