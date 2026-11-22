"""URDF forward kinematics regression; activate Conda and source the workspace."""

import unittest
import xml.etree.ElementTree as ET

try:
    from arm_control.poses import load_named_poses
    from arm_description.model import RobotModel
    from arm_description.resources import package_file
    from arm_description import JOINT_NAMES
    import xacro
except ImportError as exc:
    raise unittest.SkipTest("Activate Conda and source install/setup.bash") from exc


class RobotModelTest(unittest.TestCase):
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
