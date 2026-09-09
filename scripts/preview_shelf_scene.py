"""Render the existing shelf task's initial scene without planning a rollout.

Run with the RoboTwin environment's Python. The preview uses the task's own
geometry, robot, physics, and cameras; only motion-planner setup is omitted.
"""

import argparse
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np
from PIL import Image
import sapien
import yaml

from envs.robot import Robot
from envs.sort_spool_blocks_on_shelf import sort_spool_blocks_on_shelf


class ShelfPreview(sort_spool_blocks_on_shelf):
    def load_robot(self, **kwargs):
        self.robot = Robot(self.scene, need_topp=False, **kwargs)
        self.robot.init_joints()
        for entity in (self.robot.left_entity, self.robot.right_entity):
            for link in entity.get_links():
                link.set_mass(1)

    def together_open_gripper(self, **kwargs):
        # Initialization only: drive the grippers directly without a planner.
        for _ in range(500):
            self.robot.set_gripper(1.0, "left")
            self.robot.set_gripper(1.0, "right")
            self.scene.step()


def add_view(scene, name, position, target, fovy):
    position = np.asarray(position, dtype=float)
    forward = np.asarray(target, dtype=float) - position
    forward /= np.linalg.norm(forward)
    left = np.cross([0, 0, 1], forward)
    left /= np.linalg.norm(left)
    up = np.cross(forward, left)
    matrix = np.eye(4)
    matrix[:3, :3] = np.column_stack((forward, left, up))
    matrix[:3, 3] = position
    camera = scene.add_camera(name, 1280, 960, np.deg2rad(fovy), 0.01, 100)
    camera.entity.set_pose(sapien.Pose(matrix))
    return camera


def save_view(camera, path):
    camera.take_picture()
    pixels = (camera.get_picture("Color")[..., :3] * 255).clip(0, 255).astype(np.uint8)
    Image.fromarray(pixels).save(path)
    print(f"Saved {path}", flush=True)


def build_scene(seed):
    os.chdir(ROOT)
    config_root = ROOT / "env_cfg" / "task_config"
    config = yaml.safe_load((config_root / "shelf_preview.yml").read_text())
    embodiments = yaml.safe_load((config_root / "_embodiment_config.yml").read_text())
    robot_path = embodiments[config["embodiment"][0]]["file_path"]
    robot_config = yaml.safe_load((ROOT / robot_path / "config.yml").read_text())
    config.update(
        task_name="sort_spool_blocks_on_shelf",
        render_freq=0,
        save_data=False,
        left_robot_file=robot_path,
        right_robot_file=robot_path,
        left_embodiment_config=robot_config,
        right_embodiment_config=robot_config,
        dual_arm_embodied=True,
    )

    task = ShelfPreview()
    print(f"Building shelf scene, seed={seed}", flush=True)
    task.setup_demo(seed=seed, **config)
    return task


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output", type=Path, default=ROOT / "result" / "shelf_preview")
    options = parser.parse_args()
    output = options.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    task = build_scene(options.seed)
    views = {
        "overview": add_view(task.scene, "preview_overview", [1.35, -1.90, 1.75], [0, -0.28, 0.65], 55),
        "shelf_detail": add_view(task.scene, "preview_detail", [0, -0.43, 0.95], [0, 0.06, 0.94], 110),
        "lower_level": add_view(task.scene, "preview_lower", [0, -0.40, 0.86], [0, 0.08, 0.83], 64),
    }
    task._update_render()
    for name, camera in views.items():
        save_view(camera, output / f"{name}.png")
    task.cameras.update_picture()
    for name, data in task.cameras.get_rgb().items():
        path = output / f"{name}.png"
        Image.fromarray(data["rgb"]).save(path)
        print(f"Saved {path}", flush=True)

    head_camera = next(camera for camera in task.cameras.static_camera_list if camera.get_name() == "head_camera")
    head_labels = head_camera.get_picture("Segmentation")[..., 1]
    lower_labels = views["lower_level"].get_picture("Segmentation")[..., 1]
    visibility = {
        name: {
            "head_pixels": int(np.count_nonzero(head_labels == block.actor.per_scene_id)),
            "lower_view_pixels": int(np.count_nonzero(lower_labels == block.actor.per_scene_id)),
        }
        for name, block in task.blocks.items()
    }
    print("Block visibility:", json.dumps(visibility), flush=True)

    metadata = {
        "task": task.task_name,
        "seed": options.seed,
        "state": "initial scene after physics settling; no expert rollout",
        "blocks": {
            name: {"color": task.block_colors[name], "position": block.get_pose().p.tolist(),
                   "quaternion_wxyz": block.get_pose().q.tolist()}
            for name, block in task.blocks.items()
        },
        "shelf_floor_top": task.shelf_floor_top,
        "lower_floor_top": task.lower_floor_top,
        "block_visibility": visibility,
    }
    (output / "scene.json").write_text(json.dumps(metadata, indent=2) + "\n")
    task.close_env()


if __name__ == "__main__":
    main()
