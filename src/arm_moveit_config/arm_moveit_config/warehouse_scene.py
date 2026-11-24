"""A steel factory workcell shared by Gazebo, MoveIt and the stock planner."""

import copy
import random

from arm_description.resources import load_yaml
from moveit_msgs.msg import ObjectColor
from std_msgs.msg import ColorRGBA

from .shelf_scene import ShelfScene, collision_box, pose, scene_diff


PALETTE = {
    "red": (0.95, 0.045, 0.025),
    "blue": (0.04, 0.33, 0.95),
    "green": (0.06, 0.78, 0.25),
}
STEEL = (0.23, 0.29, 0.34)
METAL = (0.57, 0.64, 0.69)
DARK = (0.075, 0.10, 0.13)
YELLOW = (0.98, 0.70, 0.06)


class WarehouseScene(ShelfScene):
    object_id = "stock_item_0"

    def __init__(self, config=None):
        super().__init__(copy.deepcopy(config or load_yaml("arm_moveit_config", "config/warehouse.yaml")))
        self.warehouse = self.config["warehouse"]
        self.dynamic_ids = tuple(f"stock_item_{index}" for index in range(6))
        self.locations = {item: f"stock_{index // 2}_{index % 2}"
                          for index, item in enumerate(self.dynamic_ids)}
        self.slot_centers = {}
        for lane, y in enumerate(self.warehouse["lanes_y"]):
            for row, x in enumerate(self.warehouse["rows_x"]):
                self.slot_centers[f"stock_{lane}_{row}"] = [x, y, self.config["shelf"]["surface_z"] + 0.02]
        buffer = self.warehouse["buffer"]
        for index, center in enumerate(buffer["centers"]):
            self.slot_centers[f"buffer_{index}"] = [*center, buffer["surface_z"] + 0.032]
        self.requirements = {}
        for order in self.warehouse["orders"]:
            for index, color in enumerate(order["colors"]):
                key = f"order_{order['id']}_{index}"
                self.requirements[key] = color
                self.slot_centers[key] = [order["center_xy"][0] + (-0.06 if index == 0 else 0.06),
                                          order["center_xy"][1], self.config["box"]["base_z"] + 0.032]
        self.colors = {}
        self.centers = {}
        self.orientations = {}
        self.destination = "order_A_0"
        self.source = self.locations[self.object_id]
        self.randomize(0)

    def randomize(self, seed):
        rng = random.Random(None if seed < 0 else seed)
        lanes = list(range(3))
        rng.shuffle(lanes)
        # Two parts of each color; green parts go to the existing open tray.
        assignment = {lanes[0]: ("green", "red"), lanes[1]: ("green", "blue"),
                      lanes[2]: ("blue", "red")}
        jitter = self.warehouse["spawn_jitter"] if self.config["perception"]["randomize_spawn"] else 0.0
        for index, item in enumerate(self.dynamic_ids):
            self.colors[item] = assignment[index // 2][index % 2]
            center = list(self.slot_centers[self.locations[item]])
            center[:2] = [value + rng.uniform(-jitter, jitter) for value in center[:2]]
            self.centers[item] = center

    @property
    def spawn_positions(self):
        return self.centers

    def select(self, item, destination):
        self.object_id = item
        self.source, self.destination = self.locations[item], destination
        self.pick_center = list(self.centers[item])
        self.place_center = list(self.slot_centers[destination])
        self.place_center[2] += self.motion["place_clearance"]
        # A per-slot acceptance box checks the actual landing, including tilt.
        self.config["box"] = {
            "center_xy": self.slot_centers[destination][:2],
            "size_xy": [0.12, 0.14], "thickness": 0.012, "height": 0.08,
            "base_z": self.slot_centers[destination][2] - 0.032,
        }

    def payload_color(self):
        r, g, b = PALETTE[self.colors[self.object_id]]
        return ObjectColor(id=self.object_id, color=ColorRGBA(r=r, g=g, b=b, a=1.0))

    def attach(self):
        result = super().attach()
        result.object_colors = [self.payload_color()]
        return result

    def detach(self, center):
        result = super().detach(center)
        result.object_colors = [self.payload_color()]
        return result

    def initial(self):
        result = scene_diff()
        result.robot_state.attached_collision_objects = [self.attachment(remove=True)]

        def add(name, size, center, color):
            result.world.collision_objects.append(collision_box(name, size, center, self.frame))
            result.object_colors.append(ObjectColor(id=name, color=ColorRGBA(
                r=float(color[0]), g=float(color[1]), b=float(color[2]), a=1.0)))

        # Epoxy floor, expansion joints and a marked robot-cell boundary.
        add("factory_floor", [3.6, 3.2, 0.03], [0, 0, -0.024], (0.20, 0.25, 0.28))
        for index in range(-3, 4):
            add(f"floor_joint_x_{index}", [0.006, 3.2, 0.001], [index * 0.5, 0, -0.008], DARK)
            add(f"floor_joint_y_{index}", [3.6, 0.006, 0.001], [0, index * 0.5, -0.008], DARK)
        for side in (-1, 1):
            for index in range(16):
                along = -1.05 + index * 0.14
                color = YELLOW if index % 2 == 0 else DARK
                add(f"safety_x_{side}_{index}", [0.125, 0.045, 0.003], [along, side * 0.88, -0.005], color)
                add(f"safety_y_{side}_{index}", [0.045, 0.10, 0.003], [side * 1.12, along * 0.78, -0.005], color)

        # Open stock table with all supports below the picking surface.
        s = self.config["shelf"]
        x, y = s["center_xy"]
        dx, dy = s["size_xy"]
        add("stock_deck", [dx, dy, 0.02], [x, y, s["surface_z"] - 0.01], METAL)
        leg_height = s["surface_z"] - 0.02
        for ix in (-1, 1):
            for iy in (-1, 1):
                leg_x, leg_y = x + ix * (dx / 2 - 0.03), y + iy * (dy / 2 - 0.03)
                add(f"stock_table_leg_{ix}_{iy}", [0.028, 0.028, leg_height],
                    [leg_x, leg_y, leg_height / 2], STEEL)
                add(f"stock_table_foot_{ix}_{iy}", [0.065, 0.065, 0.012],
                    [leg_x, leg_y, 0.006], DARK)

        def bench(name, center, size, height):
            bx, by = center
            wx, wy = size
            add(name + "_top", [wx, wy, 0.025], [bx, by, height - 0.0125], METAL)
            for ix in (-1, 1):
                for iy in (-1, 1):
                    add(f"{name}_leg_{ix}_{iy}", [0.028, 0.028, height - 0.025],
                        [bx + ix * (wx / 2 - 0.03), by + iy * (wy / 2 - 0.03), (height - 0.025) / 2], STEEL)
            add(name + "_apron", [0.03, wy, 0.08], [bx - wx / 2 + 0.015, by, height - 0.065], DARK)

        bench("packing_bench", [-0.43, -0.065], [0.38, 0.78], 0.22)
        bench("buffer_bench", [0.195, -0.43], [0.36, 0.22], 0.22)
        for order in self.warehouse["orders"]:
            bx, by = order["center_xy"]
            name = f"tray_{order['id']}"
            color = PALETTE[order["colors"][0]]
            add(name + "_bottom", [0.28, 0.26, 0.012], [bx, by, 0.226], DARK)
            for sign in (-1, 1):
                add(f"{name}_wall_x_{sign}", [0.012, 0.26, 0.065], [bx + sign * 0.134, by, 0.2525], color)
                add(f"{name}_wall_y_{sign}", [0.28, 0.012, 0.065], [bx, by + sign * 0.124, 0.2525], color)
            add(name + "_divider", [0.008, 0.236, 0.035], [bx, by, 0.2495], color)
        for index, center in enumerate(self.warehouse["buffer"]["centers"]):
            add(f"buffer_pad_{index}", [0.12, 0.14, 0.012], [*center, 0.226], DARK)
            for sign in (-1, 1):
                add(f"buffer_mark_{index}_{sign}", [0.12, 0.006, 0.002], [center[0], center[1] + sign * 0.067, 0.233], PALETTE["green"])

        # Rear safety fence: dark mesh with yellow posts, clear front view.
        for index, fx in enumerate((-1.10, 0.0, 1.10)):
            add(f"fence_post_{index}", [0.045, 0.045, 1.02], [fx, 0.90, 0.51], YELLOW)
        for z in (0.12, 0.95):
            add(f"fence_rail_{z}", [2.2, 0.025, 0.025], [0, 0.90, z], DARK)
        for index in range(23):
            add(f"fence_mesh_{index}", [0.005, 0.008, 0.82], [-1.07 + index * 0.097, 0.90, 0.535], STEEL)
        for index in range(7):
            add(f"fence_mesh_horizontal_{index}", [2.16, 0.008, 0.005], [0, 0.90, 0.18 + index * 0.115], STEEL)

        # Service cabinet and operator terminal beyond the working envelope.
        add("control_cabinet", [0.32, 0.26, 0.72], [0.88, 0.64, 0.36], (0.72, 0.76, 0.78))
        add("cabinet_door", [0.29, 0.008, 0.65], [0.88, 0.506, 0.36], STEEL)
        add("cabinet_handle", [0.015, 0.018, 0.10], [0.98, 0.491, 0.42], DARK)
        for index in range(6):
            add(f"cabinet_vent_{index}", [0.20, 0.004, 0.008], [0.88, 0.499, 0.13 + index * 0.022], DARK)
        add("terminal_foot", [0.23, 0.20, 0.025], [-0.82, -0.62, 0.0125], STEEL)
        add("terminal_post", [0.045, 0.045, 0.65], [-0.82, -0.62, 0.35], METAL)
        add("terminal_housing", [0.25, 0.045, 0.18], [-0.82, -0.62, 0.75], DARK)
        add("terminal_screen", [0.20, 0.005, 0.125], [-0.82, -0.646, 0.765], (0.03, 0.46, 0.55))
        add("terminal_stop", [0.025, 0.014, 0.025], [-0.74, -0.655, 0.695], (0.9, 0.04, 0.04))
        add("tower_pole", [0.018, 0.018, 0.28], [0.88, 0.64, 0.86], DARK)
        for index, color in enumerate(((0.5, 0.06, 0.04), YELLOW, (0.06, 0.9, 0.38))):
            add(f"tower_light_{index}", [0.045, 0.045, 0.045], [0.88, 0.64, 0.99 + index * 0.05], color)

        from arm_perception.geometry import quaternion_from_rpy
        from arm_perception.mounting import camera_fixtures
        camera_config = load_yaml("arm_perception", "config/warehouse_camera.yaml")
        for fixture in camera_fixtures(camera_config):
            add(fixture["name"], fixture["size"], fixture["center"], fixture["color"])
            q = result.world.collision_objects[-1].primitive_poses[0].orientation
            q.x, q.y, q.z, q.w = quaternion_from_rpy(*fixture["rpy"])
            result.object_colors[-1].color.a = 0.0
        for item in self.dynamic_ids:
            add(item, self.config["object"]["size"], self.centers[item], PALETTE[self.colors[item]])
            if item in self.orientations:
                result.world.collision_objects[-1].primitive_poses[0].orientation = copy.deepcopy(self.orientations[item])
        return result

    def stages(self):
        stages = super().stages()
        if self.source.startswith("stock_"):
            return stages
        # Buffer pickups have free vertical access and require no rack withdrawal.
        x, y, z = self.pick_center
        z += self.offset
        return [(name, kind, pose(x, y, z + self.motion["approach"], down=True)
                 if name == "outside_shelf" else
                 pose(x, y, self.motion["transfer_z"], down=True)
                 if name == "leave_shelf" else target)
                for name, kind, target in stages]
