"""Privileged evaluator. Its outputs never flow back into control decisions."""
import numpy as np
from scipy.spatial.transform import Rotation


class Evaluator:
    def __init__(self, environment):
        self.env = environment
        self.config = environment.config['evaluation']
        self.since = {'right': None, 'left': None, 'all': None}
        self.times = {'right': None, 'left': None, 'all': None}
        self.current = {'right': False, 'left': False, 'all': False}

    def sample(self, t, right_released):
        env, c = self.env, self.config
        m = env.module.get_pose().to_transformation_matrix()
        s = env.socket.get_pose().to_transformation_matrix()
        goal = s[:3, 3] + [0, 0, env.config['module_half_size'][2]]
        delta = m[:3, 3] - goal
        angle = np.rad2deg(Rotation.from_matrix(s[:3,:3].T @ m[:3,:3]).magnitude())
        lv = np.linalg.norm(env.module_body.linear_velocity)
        av = np.linalg.norm(env.module_body.angular_velocity)
        joints=env.robot.right_entity.get_active_joints()
        gripper_ids=[joints.index(j[0]) for j in env.robot.right_gripper]
        actually_released=bool(np.all(env.robot.right_entity.get_qpos()[gripper_ids]>.025))
        right = bool(np.all(abs(delta[:2]) < c['position_xy']) and abs(delta[2]) < c['position_z']
                     and angle < c['angle_degrees'] and right_released and actually_released
                     and lv < c['linear_speed'] and av < c['angular_speed'])
        q = float(env.drawer.get_qpos()[0]) if env.include_drawer else 0.
        v = float(env.drawer.get_qvel()[0]) if env.include_drawer else 0.
        left = abs(q) < c['drawer_closed'] and abs(v) < c['drawer_speed']
        for side, ok in [('right', right), ('left', left), ('all', right and left)]:
            if ok:
                if self.since[side] is None:
                    self.since[side] = t
                self.current[side] = t - self.since[side] >= c['stable_seconds'] - 1e-8
                if self.current[side] and self.times[side] is None:
                    self.times[side] = t
            else:
                self.since[side], self.times[side], self.current[side] = None, None, False
        return dict(timestamp=t, module_error_xyz=delta.tolist(), module_angle_degrees=float(angle),
                    module_linear_speed=float(lv), module_angular_speed=float(av), drawer_position=q,
                    drawer_velocity=v, predicates={'right': right, 'left': left}, stable=self.current.copy())

    def estimate_error(self, detection):
        if detection.T_world_socket is None:
            return None
        return dict(timestamp=detection.timestamp, camera=detection.camera,
                    translation_error=(detection.T_world_socket[:3,3]-self.env.socket.get_pose().p).tolist())
