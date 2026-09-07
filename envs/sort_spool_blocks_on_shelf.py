from ._base_task import Base_Task
from .utils import *
import sapien
import numpy as np
import transforms3d as t3d


class sort_spool_blocks_on_shelf(Base_Task):
    COLORS = {
        "red": (0.85, 0.08, 0.06),
        "blue": (0.05, 0.25, 0.85),
        "green": (0.08, 0.62, 0.16),
    }
    REGION_X = {"red": -0.22, "blue": 0.0, "green": 0.22}
    SHELF_Y = 0.17
    BLOCK_HALF_HEIGHT = 0.025

    def setup_demo(self, is_test=False, **kwags):
        super()._init_task_env_(**kwags)

    def _create_shelf(self):
        table_top = 0.74 + self.table_z_bias
        floor_top = table_top + 0.10
        shelf_center_z = floor_top + 0.09
        shelf_color = (0.32, 0.34, 0.36)

        builder = self.scene.create_actor_builder()
        builder.set_physx_body_type("static")
        shelf_parts = [
            # floor, top, back, outer walls, and two compartment dividers
            (sapien.Pose([0, self.SHELF_Y, floor_top - 0.012]), [0.34, 0.12, 0.012]),
            (sapien.Pose([0, self.SHELF_Y, floor_top + 0.192]), [0.34, 0.12, 0.012]),
            (sapien.Pose([0, self.SHELF_Y + 0.11, shelf_center_z]), [0.34, 0.01, 0.09]),
            (sapien.Pose([-0.33, self.SHELF_Y, shelf_center_z]), [0.01, 0.12, 0.09]),
            (sapien.Pose([0.33, self.SHELF_Y, shelf_center_z]), [0.01, 0.12, 0.09]),
            (sapien.Pose([-0.11, self.SHELF_Y, shelf_center_z]), [0.008, 0.12, 0.09]),
            (sapien.Pose([0.11, self.SHELF_Y, shelf_center_z]), [0.008, 0.12, 0.09]),
            # two feet keep the raised shelf physically connected to the table
            (sapien.Pose([-0.30, self.SHELF_Y + 0.07, table_top + 0.05]), [0.02, 0.04, 0.05]),
            (sapien.Pose([0.30, self.SHELF_Y + 0.07, table_top + 0.05]), [0.02, 0.04, 0.05]),
        ]
        for local_pose, half_size in shelf_parts:
            builder.add_box_collision(
                pose=local_pose,
                half_size=half_size,
                material=self.scene.default_physical_material,
            )
            builder.add_box_visual(pose=local_pose, half_size=half_size, material=shelf_color)

        builder.set_initial_pose(sapien.Pose())
        self.shelf = builder.build(name="three_color_shelf")
        self.shelf_floor_top = floor_top

        for color_name, x in self.REGION_X.items():
            color = self.COLORS[color_name]
            create_visual_box(
                scene=self.scene,
                pose=sapien.Pose([x, self.SHELF_Y - 0.01, floor_top + 0.003]),
                half_size=[0.085, 0.075, 0.002],
                color=color,
                name=f"{color_name}_target_region",
            )
            create_visual_box(
                scene=self.scene,
                pose=sapien.Pose([x, self.SHELF_Y + 0.099, shelf_center_z]),
                half_size=[0.085, 0.002, 0.065],
                color=color,
                name=f"{color_name}_region_back",
            )

    def _create_spool_block(self, color_name, pose):
        color = self.COLORS[color_name]
        builder = self.scene.create_actor_builder()
        builder.set_physx_body_type("dynamic")

        # Square caps hide the yaw from a top view; the recessed rectangular
        # waist exposes the only reliable grasp direction from a side view.
        parts = [
            (sapien.Pose([0, 0, -0.018]), [0.034, 0.034, 0.007]),
            (sapien.Pose([0, 0, 0]), [0.010, 0.026, 0.017]),
            (sapien.Pose([0, 0, 0.018]), [0.034, 0.034, 0.007]),
        ]
        for local_pose, half_size in parts:
            builder.add_box_collision(
                pose=local_pose,
                half_size=half_size,
                material=self.scene.default_physical_material,
            )
            builder.add_box_visual(pose=local_pose, half_size=half_size, material=color)

        builder.set_initial_pose(pose)
        entity = builder.build(name=f"{color_name}_spool_block")
        actor_data = {
            "center": [0, 0, 0],
            "extents": [0.068, 0.068, 0.05],
            "scale": [1, 1, 1],
            "target_pose": [np.eye(4).tolist()],
            "contact_points_pose": [
                [[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]],
                [[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]],
            ],
            "functional_matrix": [
                [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -self.BLOCK_HALF_HEIGHT], [0, 0, 0, 1]],
            ],
            "contact_points_group": [[0, 1]],
            "contact_points_mask": [True],
        }
        return Actor(entity, actor_data, mass=0.04)

    def load_actors(self):
        self._create_shelf()

        table_top = 0.74 + self.table_z_bias
        block_z = table_top + self.BLOCK_HALF_HEIGHT
        positions = [(-0.18, -0.16), (0.0, -0.19), (0.18, -0.16)]
        permutations = [
            (1, 2, 0),
            (2, 0, 1),
            (0, 2, 1),
            (2, 1, 0),
            (1, 0, 2),
        ]
        permutation = permutations[np.random.randint(0, len(permutations))]

        self.blocks = {}
        for color_name, position_id in zip(self.COLORS, permutation):
            x, y = positions[position_id]
            yaw = np.random.uniform(-np.pi, np.pi)
            pose = sapien.Pose(
                [x, y, block_z],
                t3d.euler.euler2quat(0, 0, yaw),
            )
            block = self._create_spool_block(color_name, pose)
            self.blocks[color_name] = block
            self.add_prohibit_area(block, padding=0.035)

        self.prohibited_area.append([-0.35, 0.04, 0.35, 0.30])

    def _move_observer_into_view(self, observer_arm):
        x_shift = 0.10 if observer_arm == "left" else -0.10
        return self.move_by_displacement(
            arm_tag=observer_arm,
            x=x_shift,
            y=0.08,
            z=0.08,
            quat=GRASP_DIRECTION_DIC["top_down"],
        )

    def play_once(self):
        blocks_in_pick_order = sorted(self.blocks.items(), key=lambda item: item[1].get_pose().p[0])

        for color_name, block in blocks_in_pick_order:
            grasp_arm = ArmTag("left" if block.get_pose().p[0] <= 0 else "right")
            observer_arm = grasp_arm.opposite

            # Move the opposite wrist camera above the mixed blocks before the
            # grasping arm commits to the narrow, yaw-dependent grasp.
            self.move(self._move_observer_into_view(observer_arm))
            self.delay(1)

            self.move(
                self.grasp_actor(
                    block,
                    arm_tag=grasp_arm,
                    pre_grasp_dis=0.08,
                    contact_point_id=[0, 1],
                )
            )
            self.move(
                self.move_by_displacement(grasp_arm, z=0.12),
                self.back_to_origin(observer_arm),
            )

            target_pose = [
                self.REGION_X[color_name],
                self.SHELF_Y - 0.01,
                self.shelf_floor_top + self.BLOCK_HALF_HEIGHT,
                1,
                0,
                0,
                0,
            ]
            self.move(
                self.place_actor(
                    block,
                    arm_tag=grasp_arm,
                    target_pose=target_pose,
                    pre_dis=0.10,
                    dis=0.015,
                    constrain="free",
                    pre_dis_axis=[0, 1, 0],
                )
            )
            self.move(self.move_by_displacement(grasp_arm, y=-0.08, z=0.05))
            self.move(self.back_to_origin(grasp_arm))

        self.delay(2)
        self.info["info"] = {
            "{A}": "red spool block",
            "{B}": "blue spool block",
            "{C}": "green spool block",
            "{D}": "three-color shelf",
        }
        return self.info

    def check_success(self):
        target_y = self.SHELF_Y - 0.01
        target_z = self.shelf_floor_top + self.BLOCK_HALF_HEIGHT

        for color_name, block in self.blocks.items():
            block_position = block.get_pose().p
            in_matching_region = (
                abs(block_position[0] - self.REGION_X[color_name]) < 0.075
                and abs(block_position[1] - target_y) < 0.075
                and abs(block_position[2] - target_z) < 0.045
            )
            if not in_matching_region:
                return False
            if self.check_actors_contact(block.get_name(), "table"):
                return False
            if not self.check_actors_contact(block.get_name(), "three_color_shelf"):
                return False

        return self.is_left_gripper_open() and self.is_right_gripper_open()
