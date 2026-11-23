"""Joint limits and forward kinematics derived from the robot's URDF."""

from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

from arm_description import JOINT_NAMES
from arm_description.resources import package_file
import numpy as np
import xacro


@dataclass(frozen=True)
class JointLimit:
    lower: float
    upper: float
    velocity: float


@dataclass(frozen=True)
class TcpPosition:
    x: float
    y: float
    z: float

    @property
    def radius(self):
        return math.hypot(self.x, self.y)


@dataclass(frozen=True)
class ChainJoint:
    name: str
    kind: str
    origin: np.ndarray
    axis: np.ndarray


def origin_transform(element):
    matrix = np.eye(4)
    if element is None:
        return matrix
    matrix[:3, 3] = [float(value) for value in element.get("xyz", "0 0 0").split()]
    roll, pitch, yaw = (float(value) for value in element.get("rpy", "0 0 0").split())
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    matrix[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    return matrix


def axis_rotation(axis, angle):
    """Rodrigues' rotation formula for a normalized URDF joint axis."""
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    cosine, sine = math.cos(angle), math.sin(angle)
    return cosine * np.eye(3) + (1 - cosine) * np.outer(axis, axis) + sine * skew


@dataclass
class RobotModel:
    limits: dict[str, JointLimit]
    chain: tuple[ChainJoint, ...]

    @classmethod
    def load(cls, *, root_frame="base_footprint", tip_frame="tcp"):
        path = package_file("arm_description", "urdf/arm.urdf.xacro")
        # Parse Xacro once; both limits and kinematics use the same document.
        root = ET.fromstring(xacro.process_file(str(path)).toxml())
        joints = root.findall("joint")
        limits = {}
        for joint in joints:
            name = joint.get("name")
            if name not in JOINT_NAMES:
                continue
            limit = joint.find("limit")
            if limit is None:
                raise ValueError(f"Joint '{name}' is missing a <limit> tag in {path}.")
            limits[name] = JointLimit(
                *(float(limit.get(key)) for key in ("lower", "upper", "velocity"))
            )
        missing = set(JOINT_NAMES) - limits.keys()
        if missing:
            raise ValueError(f"Missing joint limits for: {', '.join(sorted(missing))}")

        # Walk from child to parent so additional branches cannot overwrite the TCP chain.
        by_child = {joint.find("child").get("link"): joint for joint in joints}
        chain, visited = [], set()
        current = tip_frame
        while current != root_frame:
            if current in visited or current not in by_child:
                raise ValueError(f"Could not find a joint chain from {root_frame} to {tip_frame}.")
            visited.add(current)
            joint = by_child[current]
            kind = joint.get("type")
            if kind not in ("fixed", "revolute", "continuous"):
                raise ValueError(f"Unsupported joint type in TCP chain: {kind}")
            axis_element = joint.find("axis")
            axis = np.array([float(value) for value in (
                axis_element.get("xyz", "0 0 1") if axis_element is not None else "0 0 1"
            ).split()])
            norm = np.linalg.norm(axis)
            if norm == 0:
                raise ValueError("Joint axis vector must be non-zero.")
            chain.append(ChainJoint(
                joint.get("name"), kind, origin_transform(joint.find("origin")), axis / norm,
            ))
            current = joint.find("parent").get("link")
        return cls(limits, tuple(reversed(chain)))

    def tcp_transform(self, positions) -> np.ndarray:
        angles = dict(zip(JOINT_NAMES, positions))
        transform = np.eye(4)
        for joint in self.chain:
            transform = transform @ joint.origin
            if joint.kind != "fixed":
                transform[:3, :3] = (
                    transform[:3, :3] @ axis_rotation(joint.axis, angles[joint.name])
                )
        return transform

    def tcp_position(self, positions) -> TcpPosition:
        return TcpPosition(*(float(value) for value in self.tcp_transform(positions)[:3, 3]))
