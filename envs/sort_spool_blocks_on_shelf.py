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
    SHELF_Y = 0.06
    SHELF_FRONT = -0.14
    SHELF_BACK = 0.26
    UPPER_FRONT = -0.22
    BLOCKS_PER_COLOR = 3
    BLOCK_HALF_HEIGHT = 0.025

    def setup_demo(self, is_test=False, **kwags):
        super()._init_task_env_(**kwags)

    def _create_shelf(self):
        table_top = 0.74 + self.table_z_bias
        self.lower_floor_top = table_top + 0.04
        self.shelf_floor_top = self.lower_floor_top + 0.16
        roof_bottom = self.shelf_floor_top + 0.16
        shelf_center_z = (self.lower_floor_top + roof_bottom) / 2
        upper_center_z = self.shelf_floor_top + 0.08
        shelf_color = (0.32, 0.34, 0.36)

        builder = self.scene.create_actor_builder()
        builder.set_physx_body_type("static")
        shelf_parts = [
            # One undivided lower bay. The middle board overhangs its opening
            # by 8 cm, physically hiding the blocks from the elevated head view.
            (sapien.Pose([0, self.SHELF_Y, self.lower_floor_top - 0.012]), [0.34, 0.20, 0.012]),
            (sapien.Pose([0, (self.UPPER_FRONT + self.SHELF_BACK) / 2, self.shelf_floor_top - 0.012]),
             [0.34, (self.SHELF_BACK - self.UPPER_FRONT) / 2, 0.012]),
            (sapien.Pose([0, self.SHELF_Y, roof_bottom + 0.012]), [0.34, 0.20, 0.012]),
            (sapien.Pose([0, self.SHELF_BACK - 0.01, shelf_center_z]), [0.34, 0.01, 0.16]),
            (sapien.Pose([-0.33, self.SHELF_Y, shelf_center_z]), [0.01, 0.20, 0.16]),
            (sapien.Pose([0.33, self.SHELF_Y, shelf_center_z]), [0.01, 0.20, 0.16]),
            # Dividers exist only on the upper level.
            (sapien.Pose([-0.11, self.SHELF_Y, upper_center_z]), [0.008, 0.20, 0.08]),
            (sapien.Pose([0.11, self.SHELF_Y, upper_center_z]), [0.008, 0.20, 0.08]),
            (sapien.Pose([-0.30, self.SHELF_Y, table_top + 0.008]), [0.02, 0.12, 0.008]),
            (sapien.Pose([0.30, self.SHELF_Y, table_top + 0.008]), [0.02, 0.12, 0.008]),
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

        for color_name, x in self.REGION_X.items():
            color = self.COLORS[color_name]
            create_visual_box(
                scene=self.scene,
                pose=sapien.Pose([x, self.SHELF_Y, self.shelf_floor_top + 0.001]),
                half_size=[0.09, 0.175, 0.001],
                color=color,
                name=f"{color_name}_target_region",
            )
            create_visual_box(
                scene=self.scene,
                pose=sapien.Pose([x, self.SHELF_BACK - 0.022, upper_center_z]),
                half_size=[0.085, 0.002, 0.065],
                color=color,
                name=f"{color_name}_region_back",
            )

    def _create_spool_block(self, color_name, pose, name):
        color = self.COLORS[color_name]
        builder = self.scene.create_actor_builder()
        builder.set_physx_body_type("dynamic")

        # The narrow waist can be pinched horizontally through the lower bay.
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
        entity = builder.build(name=name)
        actor_data = {
            "center": [0, 0, 0],
            "extents": [0.068, 0.068, 0.05],
            "scale": [1, 1, 1],
            "target_pose": [np.eye(4).tolist()],
            "contact_points_pose": [
                [[0, 0, -1, 0], [0, -1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]],
                [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]],
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

        block_z = self.lower_floor_top + self.BLOCK_HALF_HEIGHT
        positions = [(x, y) for y in (-0.025, 0.08, 0.185) for x in (-0.23, 0.0, 0.23)]
        colors = np.random.permutation(list(self.COLORS) * self.BLOCKS_PER_COLOR)

        self.blocks = {}
        self.block_colors = {}
        for index, (color_name, (x, y)) in enumerate(zip(colors, positions)):
            # Leave clearance between rows and keep side grasps accessible
            # through the front opening rather than through the shelf walls.
            x += np.random.uniform(-0.01, 0.01)
            y += np.random.uniform(-0.003, 0.003)
            yaw = np.random.uniform(-0.32, 0.32) + np.random.randint(2) * np.pi
            pose = sapien.Pose(
                [x, y, block_z],
                t3d.euler.euler2quat(0, 0, yaw),
            )
            name = f"{color_name}_spool_block_{index}"
            block = self._create_spool_block(color_name, pose, name)
            self.blocks[name] = block
            self.block_colors[name] = str(color_name)
            self.add_prohibit_area(block, padding=0.035)

        self.prohibited_area.append([-0.35, self.UPPER_FRONT - 0.04, 0.35, self.SHELF_BACK + 0.02])

    def _move_observer_into_view(self, observer_arm):
        x = -0.27 if observer_arm == "left" else 0.27
        # Compensate for the default wrist camera's mount angle and height;
        # its optical axis should look into the lower bay, not at the board.
        orientation = t3d.quaternions.mat2quat(
            t3d.euler.euler2mat(-0.46, 0, 0) @ t3d.euler.euler2mat(0, 0, np.pi / 2)
        )
        return self.move_to_pose(
            arm_tag=observer_arm,
            target_pose=[x, self.UPPER_FRONT - 0.18, self.lower_floor_top + 0.17, *orientation],
        )

    def play_once(self):
        # Clear the front row first; fill each upper compartment back to front.
        blocks_in_pick_order = sorted(self.blocks.items(), key=lambda item: (item[1].get_pose().p[1], item[1].get_pose().p[0]))
        placed_counts = dict.fromkeys(self.COLORS, 0)

        for name, block in blocks_in_pick_order:
            color_name = self.block_colors[name]
            grasp_arm = ArmTag("left" if block.get_pose().p[0] <= 0 else "right")
            observer_arm = grasp_arm.opposite

            # Inspect through the open front, below the occluding middle board.
            self.move(self._move_observer_into_view(observer_arm))
            self.delay(1)

            if not self.plan_success:
                return self.info
            contact_id = min(
                (0, 1),
                key=lambda index: self.get_grasp_pose(block, grasp_arm, contact_point_id=index, pre_dis=0.08)[1],
            )
            self.move(
                self.grasp_actor(
                    block,
                    arm_tag=grasp_arm,
                    pre_grasp_dis=block.get_pose().p[1] - (self.UPPER_FRONT - 0.08),
                    contact_point_id=[contact_id],
                )
            )
            # Pull out horizontally before lifting past the middle board.
            self.move(self.move_by_displacement(grasp_arm, z=0.008))
            withdrawal = self.UPPER_FRONT - 0.08 - block.get_pose().p[1]
            self.move(self.move_by_displacement(grasp_arm, y=withdrawal))
            self.move(self.back_to_origin(observer_arm))
            self.move(self.move_by_displacement(grasp_arm, z=0.17))

            target_pose = [
                self.REGION_X[color_name],
                0.16 - 0.11 * placed_counts[color_name],
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
                    pre_dis=target_pose[1] - (self.UPPER_FRONT - 0.08),
                    dis=0.0,
                    constrain="free",
                    pre_dis_axis=[0, 1, 0],
                )
            )
            self.move(self.move_by_displacement(grasp_arm, y=self.UPPER_FRONT - 0.08 - target_pose[1]))
            self.move(self.back_to_origin(grasp_arm))
            placed_counts[color_name] += 1

        self.delay(2)
        self.info["info"] = {
            "{A}": "three red spool blocks",
            "{B}": "three blue spool blocks",
            "{C}": "three green spool blocks",
            "{D}": "two-level shelf with color-coded upper compartments",
        }
        return self.info

    def check_success(self):
        target_z = self.shelf_floor_top + self.BLOCK_HALF_HEIGHT

        for name, block in self.blocks.items():
            color_name = self.block_colors[name]
            block_position = block.get_pose().p
            in_matching_region = (
                abs(block_position[0] - self.REGION_X[color_name]) < 0.075
                and self.SHELF_FRONT + 0.045 < block_position[1] < self.SHELF_BACK - 0.06
                and abs(block_position[2] - target_z) < 0.025
            )
            if not in_matching_region:
                return False
            if self.check_actors_contact(block.get_name(), "table"):
                return False
            if not self.check_actors_contact(block.get_name(), "three_color_shelf"):
                return False

        return self.is_left_gripper_open() and self.is_right_gripper_open()
