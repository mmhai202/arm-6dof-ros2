"""Collision geometry and task poses for a cabinet-to-floor-box task."""

import math
import xml.etree.ElementTree as ET

from arm_description import GRIPPER_TOUCH_LINKS
from arm_description.resources import load_yaml, package_file
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import (
    AttachedCollisionObject, CollisionObject, ObjectColor, PlanningScene,
)
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA, Header
import xacro


OBJECT_ID = "shelf_payload"
TOUCH_LINKS = list(GRIPPER_TOUCH_LINKS)


def position_limits():
    root = ET.fromstring(xacro.process_file(str(package_file(
        "arm_description", "urdf/arm.urdf.xacro"))).toxml())
    return {joint.get("name"): (float(joint.find("limit").get("lower")),
                                float(joint.find("limit").get("upper")))
            for joint in root.findall("joint") if joint.get("type") == "revolute"}


def pose(x, y, z, *, down=False):
    return Pose(position=Point(x=float(x), y=float(y), z=float(z)),
                orientation=Quaternion(x=1.0, w=0.0) if down else Quaternion(w=1.0))


def collision_box(name, size, center, frame):
    return CollisionObject(
        id=name, header=Header(frame_id=frame),
        pose=pose(0, 0, 0),
        primitives=[SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(map(float, size)))],
        primitive_poses=[pose(*center)], operation=CollisionObject.ADD,
    )


def scene_diff():
    result = PlanningScene(is_diff=True)
    result.robot_state.is_diff = True
    return result


class ShelfScene:
    def __init__(self, config=None):
        self.config = config or load_yaml("arm_moveit_config", "config/shelf_pick_place.yaml")
        self.frame = self.config["frame"]
        self.motion = self.config["motion"]
        s, o, b = (self.config[key] for key in ("shelf", "object", "box"))
        for values in (s["size_xy"], o["size"], b["size_xy"]):
            if not all(math.isfinite(v) and v > 0 for v in values):
                raise ValueError("Scene dimensions must be finite and positive.")
        if any(o["size"][i] >= b["size_xy"][i] - 2 * b["thickness"] for i in (0, 1)):
            raise ValueError("Payload must fit inside the box.")
        self.pick_center = [*o["center_xy"], s["surface_z"] + o["size"][2] / 2]
        self.place_center = [*b["center_xy"], b["thickness"] + o["size"][2] / 2
                             + self.motion["place_clearance"]]
        self.offset = o["size"][2] / 2 + self.motion["tcp_gap"]
        self.gripper_closed = (o["size"][0] - 0.04) / 2
        if not 0 <= self.gripper_closed < self.motion["gripper_open"] <= 0.025:
            raise ValueError("Payload width must fit the gripper's 40–90 mm opening.")

    def payload(self, center, frame=None):
        return collision_box(OBJECT_ID, self.config["object"]["size"], center, frame or self.frame)

    def attachment(self, *, remove=False):
        # TCP +Z points down; the payload remains below it by a known grasp offset.
        obj = self.payload([0, 0, self.offset], "tcp")
        if remove:
            obj = CollisionObject(id=OBJECT_ID, operation=CollisionObject.REMOVE)
        return AttachedCollisionObject(link_name="tcp", object=obj, touch_links=TOUCH_LINKS)

    def attach(self):
        result = scene_diff()
        # MoveIt removes the matching world object while attaching. An additional
        # REMOVE in the same diff would try to remove it a second time.
        result.robot_state.attached_collision_objects = [self.attachment()]
        return result

    def detach(self, center):
        result = scene_diff()
        result.robot_state.attached_collision_objects = [self.attachment(remove=True)]
        result.world.collision_objects = [self.payload(center)]
        return result

    def initial(self):
        result = self.detach(self.pick_center)

        def add(name, size, center, color):
            result.world.collision_objects.append(collision_box(name, size, center, self.frame))
            result.object_colors.append(ObjectColor(id=name, color=ColorRGBA(
                r=color[0], g=color[1], b=color[2], a=1.0)))

        add("floor", [2.4, 2.4, 0.04], [0, 0, -0.025], (0.40, 0.43, 0.46))
        s = self.config["shelf"]
        x, y = s["center_xy"]
        dx, dy = s["size_xy"]
        t, z, top = s["thickness"], s["surface_z"], s["top_z"]
        wood = (0.58, 0.35, 0.16)
        add("cabinet_shelf", [dx, dy, t], [x, y, z - t / 2], wood)
        add("cabinet_top", [dx, dy, t], [x, y, top + t / 2], wood)
        for sign, name in ((-1, "left"), (1, "right")):
            add(f"cabinet_{name}", [dx, t, top],
                [x, y + sign * (dy / 2 + t / 2), top / 2], wood)
        b = self.config["box"]
        x, y = b["center_xy"]
        dx, dy = b["size_xy"]
        t, h = b["thickness"], b["height"]
        blue = (0.12, 0.42, 0.70)
        add("box_bottom", [dx, dy, t], [x, y, t / 2], blue)
        for sign, name in ((-1, "near"), (1, "far")):
            add(f"box_{name}", [t, dy, h], [x + sign * (dx - t) / 2, y, h / 2], blue)
        for sign, name in ((-1, "left"), (1, "right")):
            add(f"box_{name}", [dx, t, h], [x, y + sign * (dy - t) / 2, h / 2], blue)
        result.object_colors.append(ObjectColor(
            id=OBJECT_ID, color=ColorRGBA(r=0.95, g=0.20, b=0.06, a=1.0)))
        return result

    def stages(self):
        x, y, z = self.pick_center
        z += self.offset
        px, py, pz = self.place_center
        pz += self.offset
        m = self.motion
        outside_x = (self.config["shelf"]["center_xy"][0]
                     - self.config["shelf"]["size_xy"][0] / 2 - m["withdraw_clearance"])
        # Only free-space transfers use OMPL; contact motions stay straight.
        return [
            ("outside_shelf", "ompl", pose(outside_x, y, z + m["approach"], down=True)),
            ("pre_grasp", "cartesian", pose(x, y, z + m["approach"], down=True)),
            ("grasp", "cartesian", pose(x, y, z, down=True)),
            ("allow_finger_contact", "scene", None),
            ("close_gripper", "gripper", (self.gripper_closed,) * 2),
            ("attach", "scene", None),
            ("lift", "cartesian", pose(x, y, z + m["lift"], down=True)),
            ("leave_shelf", "cartesian", pose(outside_x, y, z + m["lift"], down=True)),
            ("above_box", "ompl", pose(px, py, m["transfer_z"], down=True)),
            ("place", "cartesian", pose(px, py, pz, down=True)),
            ("detach", "scene", None),
            ("release_gripper", "gripper", (m["gripper_open"],) * 2),
            ("forbid_finger_contact", "scene", None),
            ("retreat", "cartesian", pose(px, py, m["transfer_z"], down=True)),
        ]
