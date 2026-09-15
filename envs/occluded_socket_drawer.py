"""Procedural SAPIEN task using RoboTwin's Base_Task and aloha-agilex Robot.

This module owns hidden state. It must never be passed to a visual policy.
Only initialization may set robot qpos/object poses or drawer qpos.
"""
from pathlib import Path
import numpy as np
import sapien
import yaml

from ._base_task import Base_Task
from .robot import Robot
from .occluded_socket.geometry import transform, look_at, R_DOWN, T_EE_MODULE

ROOT = Path(__file__).resolve().parents[1]
COLORS = [[1, 0.03, 0.03], [0.03, 1, 0.03], [0.03, 0.08, 1], [1, 1, 0.02]]


class occluded_socket_drawer(Base_Task):
    def setup_demo(self, seed=0, load_condition='LOW', config=None, gpu=0, headless=True,
                   include_drawer=True, **kwargs):
        self.config = config or yaml.safe_load((ROOT / 'env_cfg/task_config/occluded_socket_drawer.yml').read_text(encoding='utf-8'))
        self.seed, self.load_condition = seed, load_condition
        self.task_name='occluded_socket_drawer'
        self.instruction=self.config['instruction']
        self.include_drawer = include_drawer
        self.render_freq = 0 if headless else 1
        self.random_light = False
        self.gpu = gpu
        self.setup_scene()
        self.load_robot()
        self.load_actors()
        self.load_camera()
        self.steps = 0
        self.decision_started = False

    def setup_scene(self, **kwargs):
        # CPU PhysX plus one explicit rendering GPU. No cuRobo allocation.
        # 3.0.0b1 has no offscreen_only keyword; no Viewer is created headless.
        sapien.render.set_global_config(max_num_materials=5000, max_num_textures=5000)
        # SAPIEN 3.0.0b1's default raster pipeline deadlocks during both CPU
        # and CUDA camera readback on H100/H200 with recent NVIDIA drivers.
        # RoboTwin's established RT pipeline uses a different synchronization
        # path and has working get_picture/get_picture_cuda readback here.
        sapien.render.set_camera_shader_dir('rt')
        sapien.render.set_ray_tracing_samples_per_pixel(32)
        sapien.render.set_ray_tracing_path_depth(8)
        sapien.render.set_ray_tracing_denoiser('oidn')
        self.engine = sapien.Engine()
        self.renderer = sapien.SapienRenderer()
        self.scene = sapien.Scene([sapien.physx.PhysxCpuSystem(), sapien.render.RenderSystem(device=f'cuda:{self.gpu}')])
        self.scene.set_timestep(self.config['dt'])
        self.scene.set_ambient_light([0.6, 0.6, 0.6])
        self.scene.add_directional_light([0.3, 0.5, -1], [1, 1, 1], shadow=True)
        self.scene.add_point_light([0, -0.4, 1.8], [0.5, 0.5, 0.5])
        self.material = self.scene.create_physical_material(1.2, 1.0, 0)
        self.scene.default_physical_material = self.material
        self.scene.add_ground(0)
        b = self.scene.create_actor_builder()
        self.box(b, [0, 0, 0.715], [0.6, 0.36, 0.025], [.5, .53, .55])
        self.table = b.build_static('table')

    def box(self, builder, p, half, color, collision=True):
        pose = sapien.Pose(p)
        if collision:
            builder.add_box_collision(pose=pose, half_size=half, material=self.material, density=300)
        builder.add_box_visual(pose=pose, half_size=half, material=color)

    def load_robot(self, **kwargs):
        path = ROOT / 'assets/embodiments/aloha-agilex'
        config = yaml.safe_load((path / 'config.yml').read_text())
        self.robot = Robot(self.scene, need_topp=False, left_robot_file=str(path), right_robot_file=str(path),
                           left_embodiment_config=config, right_embodiment_config=config, dual_arm_embodied=True)
        self.robot.init_joints()
        # Match Base_Task's established mass and drive initialization.
        for link in self.robot.left_entity.get_links():
            link.set_mass(1)
        self.robot_config = config

    def load_actors(self):
        c = self.config
        rng = np.random.default_rng(self.seed)
        self.socket_xyz = np.array(c['socket_center'], dtype=float)
        self.socket_xyz[:2] += rng.uniform(-np.array(c['socket_random_xy']), c['socket_random_xy'])
        # Fixed casing, genuinely open from above/front/left; no solid fake socket.
        b = self.scene.create_actor_builder()
        center = np.array(c['socket_center'])
        self.box(b, center + [0, 0, -.02], [.12, .1, .01], [.22, .24, .28])
        self.box(b, center + [.112, 0, .055], [.008, .1, .065], [.22, .24, .28])
        self.box(b, center + [0, .12, .055], [.12, .008, .065], [.22, .24, .28])
        # Side-wall occlusion; overhead insertion/release volume stays collision-free.
        self.housing = b.build_static('fixed_housing')
        # Hidden random insert: bottom, four walls and rigidly attached color marks.
        b = self.scene.create_actor_builder()
        self.box(b, [0, 0, -.004], [.09, .078, .004], [.72, .72, .72])
        sx, sy = c['socket_inner_half_size']
        for x in [-sx-.004, sx+.004]:
            self.box(b, [x, 0, .006], [.004, sy+.008, .006], [.45, .45, .45])
        for y in [-sy-.004, sy+.004]:
            self.box(b, [0, y, .006], [sx, .004, .006], [.45, .45, .45])
        for p, color in zip(c['marker_points'], COLORS):
            self.box(b, p, [c['marker_radius'], c['marker_radius'], .0005], color)
        b.initial_pose = sapien.Pose(self.socket_xyz)
        self.socket = b.build_static('hidden_socket')
        b = self.scene.create_actor_builder()
        self.box(b, [0, 0, 0], c['module_half_size'], [.06, .27, .36])
        self.box(b, [0, 0, .04], c['handle_half_size'], [.9, .6, .16])
        b.initial_pose = sapien.Pose([.32, -.20, .77])
        self.module = b.build('module')
        self.module_body = self.module.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent)
        self.module_body.set_mass(.08)
        # A real support rack for the explicit self put-down fallback.
        b = self.scene.create_actor_builder()
        self.box(b, [.38, -.18, .76], [.065, .065, .02], [.4, .37, .3])
        self.rack = b.build_static('temporary_rack')
        if self.include_drawer:
            self.build_drawer()

    def build_drawer(self):
        c = self.config
        center = np.array(c['drawer_center'])
        b = self.scene.create_articulation_builder()
        base = b.create_link_builder()
        base.set_name('drawer_cabinet')
        self.box(base, [0, 0, -.025], [.11, .15, .015], [.32, .35, .4])
        self.box(base, [0, 0, -.105], [.105, .145, .055], [.32, .35, .4])
        for x in [-.112, .112]:
            self.box(base, [x, 0, .025], [.008, .15, .05], [.32, .35, .4])
        link = b.create_link_builder(base)
        link.set_name('drawer_slide')
        link.set_joint_name('drawer_prismatic')
        # Prismatic axis +x in joint frame is world -y (opening).
        rot = [np.sqrt(.5), 0, 0, -np.sqrt(.5)]
        link.set_joint_properties('prismatic', limits=[[0, c['drawer_travel']]],
                                  pose_in_parent=sapien.Pose(q=rot), pose_in_child=sapien.Pose(q=rot), friction=.15, damping=8)
        self.box(link, [0, 0, 0], [.096, .13, .009], [.64, .57, .46])
        self.box(link, [0, -.132, .034], [.105, .008, .045], [.56, .47, .35])
        self.box(link, [0, -.156, .045], [.06, .016, .012], [.82, .14, .7])
        b.initial_pose = sapien.Pose(center)
        self.drawer = b.build(fix_root_link=True)
        self.drawer.set_qpos([c['drawer_low' if self.load_condition == 'LOW' else 'drawer_high']])
        self.drawer.get_active_joints()[0].set_drive_property(0, 0)

    def load_camera(self, **kwargs):
        c = self.config
        self.camera_map = {}
        for name in ['left', 'right', 'head', 'overview']:
            cam = self.scene.add_camera(name, c['camera_width'], c['camera_height'], np.deg2rad(c['camera_fovy']), .015, 5)
            self.camera_map[name] = cam
        self.camera_map['head'].entity.set_pose(sapien.Pose(look_at(np.array([-.032, -.45, 1.35]), [0, 0, .75])))
        self.camera_map['overview'].entity.set_pose(sapien.Pose(look_at(np.array([1.05, -1.20, 1.5]), [0, -.05, .8])))
        if self.render_freq:
            from sapien.utils.viewer import Viewer
            self.viewer = Viewer()
            self.viewer.set_scene(self.scene)
            self.viewer.set_camera_xyz(1, -1.2, 1.5)
            self.viewer.set_camera_rpy(0, -.5, 2.2)

    def render(self):
        for arm in ['left', 'right']:
            self.camera_map[arm].entity.set_pose(getattr(self.robot, f'{arm}_camera').get_pose())
        self.scene.update_render()
        if self.render_freq:
            self.viewer.render()

    def step_physics(self):
        self.scene.step()
        if self.decision_started:
            self.steps += 1

    @property
    def sim_time(self):
        return self.steps * self.config['dt']

    def snapshot(self):
        return dict(socket_pose=self.socket.get_pose().to_transformation_matrix().tolist(),
                    joint_names=[j.name for j in self.robot.left_entity.get_active_joints()],
                    module_pose=self.module.get_pose().to_transformation_matrix().tolist(),
                    module_velocity=self.module_body.linear_velocity.tolist(),
                    qpos=self.robot.left_entity.get_qpos().tolist(), qvel=self.robot.left_entity.get_qvel().tolist(),
                    drawer_qpos=self.drawer.get_qpos().tolist() if self.include_drawer else None,
                    drawer_qvel=self.drawer.get_qvel().tolist() if self.include_drawer else None)

    def close_env(self, **kwargs):
        if hasattr(self, 'viewer'):
            self.viewer.close()
