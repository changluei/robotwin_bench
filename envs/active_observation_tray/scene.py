"""SAPIEN scene; owns privileged state. No tested policy may receive this object."""
from pathlib import Path
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET
from types import SimpleNamespace
import numpy as np
import sapien
import yaml
from envs.occluded_socket.geometry import look_at

ROOT = Path(__file__).resolve().parents[2]


class TrayScene:
    def setup_demo(self, config, seed=0, physics_only=False):
        self.config, self.seed, self.physics_only = config, seed, physics_only
        self.steps = 0
        self._temporary_robot = None
        systems = [sapien.physx.PhysxCpuSystem()]
        if not physics_only:
            # The default raster pipeline deadlocks on H100/H200 during camera
            # readback. This is RoboTwin's established, working render path.
            sapien.render.set_camera_shader_dir('rt')
            sapien.render.set_ray_tracing_samples_per_pixel(config.rt_samples_per_pixel)
            # This setting is the number of RT light bounces; it does not
            # create or expose a range/depth observation.
            sapien.render.set_ray_tracing_path_depth(config.rt_bounces)
            sapien.render.set_ray_tracing_denoiser('oidn')
            systems.append(sapien.render.RenderSystem())
        self.scene = sapien.Scene(systems)
        self.scene.set_timestep(config.dt)
        self.material = self.scene.create_physical_material(1.2, 1., 0.)
        self.scene.default_physical_material = self.material
        if not physics_only:
            self.scene.set_ambient_light([.6, .6, .6])
            self.scene.add_directional_light([.3, .5, -1], [1, 1, 1], shadow=True)
        self.load_robot()
        self.load_actors()
        if not physics_only:
            self.load_camera()

    def box(self, builder, p, half, color, collision=True):
        pose = sapien.Pose(p)
        if collision:
            builder.add_box_collision(pose=pose, half_size=half,
                                      material=self.material, density=300)
        if not self.physics_only:
            builder.add_box_visual(pose=pose, half_size=half, material=color)

    def load_robot(self):
        asset = ROOT / 'assets/embodiments/aloha-agilex'
        cfg = yaml.safe_load((asset / 'config.yml').read_text())
        self.original_urdf = (asset / cfg['urdf_path']).resolve()
        if self.physics_only:
            # Debug-only visual-free copy: retain every collision, inertial,
            # joint and camera mounting link. Never edit the installed asset.
            self._temporary_robot = TemporaryDirectory(prefix='tray-physics-')
            tree = ET.parse(self.original_urdf)
            for link in tree.getroot().findall('link'):
                for visual in link.findall('visual'):
                    link.remove(visual)
            for mesh in tree.getroot().iter('mesh'):
                mesh.set('filename', str((self.original_urdf.parent / mesh.get('filename')).resolve()))
            target = Path(self._temporary_robot.name) / 'robot.urdf'
            tree.write(target)
            cfg['urdf_path'] = str(target)
        if self.physics_only:
            # Robot's module-level cuRobo import initializes CUDA even when
            # unused. CPU geometry debug loads the SAME URDF/drive configuration
            # directly; the normal rendering path uses RoboTwin Robot unchanged.
            loader = self.scene.create_urdf_loader()
            loader.fix_root_link = True
            entity = loader.load(cfg['urdf_path'])
            p = cfg['robot_pose'][0]
            entity.set_root_pose(sapien.Pose(p[:3], p[3:]))
            self.robot = SimpleNamespace(left_entity=entity, right_entity=entity,
                left_urdf_path=str(self.original_urdf), left_srdf_path=str(asset/cfg['srdf_path']))
            for joint in entity.get_active_joints():
                joint.set_drive_property(cfg['joint_stiffness'], cfg['joint_damping'])
            for i, arm in enumerate(['left', 'right']):
                setattr(self.robot, arm+'_arm_joints',
                        [entity.find_joint_by_name(n) for n in cfg['arm_joints_name'][i]])
                g = cfg['gripper_name'][i]
                gripper = [(entity.find_joint_by_name(g['base']), 1., 0.)]
                gripper += [(entity.find_joint_by_name(n), m, o) for n, m, o in g['mimic']]
                setattr(self.robot, arm+'_gripper', gripper)
                for joint, _, _ in gripper:
                    joint.set_drive_property(cfg['gripper_stiffness'], cfg['gripper_damping'])
        else:
            from envs.robot import Robot
            self.robot = Robot(self.scene, need_topp=False,
                               left_robot_file=str(asset), right_robot_file=str(asset),
                               left_embodiment_config=cfg, right_embodiment_config=cfg,
                               dual_arm_embodied=True)
            self.robot.init_joints()
        for link in self.robot.left_entity.get_links():
            link.set_mass(1)  # Existing Base_Task convention.

    def load_actors(self):
        c = self.config
        rng = np.random.default_rng(self.seed)
        self.rack_xyz = np.array(c.rack_center)
        self.rack_xyz[:2] += rng.uniform(-c.rack_xy_range, c.rack_xy_range, 2)
        self.hole_xyz = self.rack_xyz.copy()
        self.hole_xyz[0] += rng.uniform(-c.lateral_range, c.lateral_range)
        self.hole_xyz[2] += .014
        self.tray_initial = np.array(c.tray_start)
        self.tray_initial[:2] += rng.uniform(-c.tray_xy_range, c.tray_xy_range, 2)
        b = self.scene.create_actor_builder()
        self.box(b, [0, 0, .715], [.6, .36, .025], [.5, .53, .55])
        self.table = b.build_static('table')
        # Wide support, with over 4 cm lateral freedom on either side of tray.
        b = self.scene.create_actor_builder()
        self.box(b, self.rack_xyz + [0, -.015, -.03], [.105, .13, .02], [.35, .38, .4])
        self.rack = b.build_static('rack_floor')
        b = self.scene.create_actor_builder()
        # Actual aperture in a wall, not a visual decal over solid collision.
        for sign in [-1, 1]:
            self.box(b, [sign*(.06+c.slot_half_width)/2, 0, 0],
                     [(.06-c.slot_half_width)/2, .008, .016], [.6, .6, .6])
        self.box(b, [0, 0, .020], [.06, .008, .004], [.6, .6, .6])
        self.box(b, [0, 0, -.020], [.06, .008, .004], [.6, .6, .6])
        # Local marks are attached to the aperture itself, inside rack.
        for x in [-.022, .022]:
            for z in [.010, .035]:
                self.box(b, [x, -.0082, z], [.0045, .0003, .0045],
                         [1, .02, .02], False)
        b.initial_pose = sapien.Pose(self.hole_xyz)
        self.slot = b.build_static('slot')
        b = self.scene.create_actor_builder()
        self.box(b, [0, .08, 0], [c.tray_half_width, .10, .007], [.1, .3, .4])
        self.box(b, [0, 0, .04], [.012, .018, .032], [.85, .55, .15])
        self.box(b, c.peg_offset, c.peg_half, [.55, .58, .55])
        # Four uniquely colored coplanar fiducials make correspondence and
        # pose observable from RGB alone. They are visual-only geometry, not
        # a segmentation or simulator-state channel.
        tray_marks = [
            ((-.030, .125, .0073), (.02, 1., .08)),   # green
            ((-.030, .175, .0073), (1., .25, .01)),   # orange
            (( .030, .125, .0073), (1., .02, 1.)),    # magenta
            (( .030, .175, .0073), (1., 1., .02)),    # yellow
        ]
        for position, color in tray_marks:
            self.box(b, position, [.007, .007, .0003], color, False)
        # A narrow neck connects the peg physically to the distal tray edge.
        self.box(b, [0, .182, .009], [.003, .006, .005], [.1, .3, .4])
        b.initial_pose = sapien.Pose(self.tray_initial)
        self.tray = b.build('tray')
        self.tray_body = self.tray.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent)
        self.tray_body.set_mass(.10)
        # Initial tray support at the same height as insertion, no attachment.
        b = self.scene.create_actor_builder()
        self.box(b, np.array(c.tray_start) + [0, .07, -.026], [.07, .10, .019], [.4, .37, .3])
        self.source = b.build_static('tray_source_support')

        # Public Task-B geometry. The right arm must physically move the first
        # N blocks; actor poses remain evaluator-only.
        self.b_blocks = []
        self.b_goal_positions = []
        for index, (start, goal) in enumerate(zip(c.b_block_starts, c.b_block_goals)):
            b = self.scene.create_actor_builder()
            self.box(b, [0, 0, 0], [.018, .018, .018], [.12, .25, .9])
            b.initial_pose = sapien.Pose(start)
            block = b.build(f'b_block_{index}')
            block.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent).set_mass(.04)
            self.b_blocks.append(block)
            self.b_goal_positions.append(np.asarray(goal, dtype=float))
            if not self.physics_only:
                marker = self.scene.create_actor_builder()
                self.box(marker, np.asarray(goal) + [0, 0, -.0185],
                         [.025, .025, .0005], [.15, .15, .35], False)
                marker.build_static(f'b_goal_{index}')

    def load_camera(self):
        self.cameras = {name: self.scene.add_camera(name, self.config.image_width,
                                                    self.config.image_height,
                                                    np.deg2rad(55), .015, 5)
                        for name in ['left', 'right', 'head', 'overview']}
        self.cameras['head'].entity.set_pose(sapien.Pose(look_at(np.array([-.032, -.45, 1.35]), [0, 0, .78])))
        self.cameras['overview'].entity.set_pose(sapien.Pose(look_at(np.array([.8, -.9, 1.35]), [-.1, 0, .8])))

    def render(self):
        if self.physics_only:
            raise RuntimeError('Phase-2 physics-only mode provides NO camera observations')
        for arm in ['left', 'right']:
            self.cameras[arm].entity.set_pose(getattr(self.robot, arm + '_camera').get_pose())
        self.scene.update_render()

    def step_physics(self):
        self.scene.step()
        self.steps += 1

    def close_env(self, **kwargs):
        if self._temporary_robot:
            self._temporary_robot.cleanup()
