"""Pure-RGB dual-arm active-observation experiment and paired benchmark."""
import hashlib
import csv
import json
import subprocess
import sys
import traceback
from dataclasses import asdict
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

from envs.occluded_socket.geometry import R_DOWN, T_EE_CAMERA, look_at, transform
from .evaluation import measure_geometry, measure_task_b
from .motion import TrayKinematics
from .scene import TrayScene
from .vision import ColorMarkerEstimator, RGBObservationAdapter


POLICIES = (
    'no-extra-observation',
    'self-observation',
    'immediate-helper',
    'helper-after-current-B-step',
    'pre-observation',
    'helper-motion-without-helper-image',
)


class OverviewRecorder:
    """Evaluation-only overview video; frames never enter a policy."""

    def __init__(self, env, path, stride=100, fps=10):
        self.env = env
        self.path = str(path)
        self.stride = int(stride)
        self.fps = int(fps)
        self.writer = None
        self.last_step = None

    def capture(self, label, force=False):
        if self.last_step == self.env.steps:
            return
        if not force and self.env.steps % self.stride:
            return
        self.env.render()
        camera = self.env.cameras['overview']
        camera.take_picture()
        rgb = (camera.get_picture('Color')[..., :3] * 255).clip(0, 255).astype(np.uint8)
        frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        cv2.putText(frame, f'{self.env.steps * self.env.config.dt:6.2f}s  {label}',
                    (14, 28), cv2.FONT_HERSHEY_SIMPLEX, .65, (30, 30, 30), 3,
                    cv2.LINE_AA)
        cv2.putText(frame, f'{self.env.steps * self.env.config.dt:6.2f}s  {label}',
                    (14, 28), cv2.FONT_HERSHEY_SIMPLEX, .65, (245, 245, 245), 1,
                    cv2.LINE_AA)
        if self.writer is None:
            height, width = frame.shape[:2]
            self.writer = cv2.VideoWriter(
                self.path, cv2.VideoWriter_fourcc(*'mp4v'), self.fps,
                (width, height))
            if not self.writer.isOpened():
                raise RuntimeError(f'cannot open video writer: {self.path}')
        self.writer.write(frame)
        self.last_step = self.env.steps

    def close(self):
        if self.writer is not None:
            self.writer.release()


class TaskBWorker:
    """Incremental physical pick/place queue for the right arm."""

    def __init__(self, kin, config, load):
        self.kin = kin
        self.config = config
        self.load = load
        self.block = 0
        self.phase = 0
        self.path = None
        self.velocity = None
        self.path_index = 0
        self.grip_index = 0
        self.opening = .04
        self.completed_steps = []

    @property
    def done(self):
        return self.block >= self.load

    @property
    def completed_blocks(self):
        return self.block

    def invalidate_motion(self):
        if self.phase not in (2, 6):
            self.path = None
            self.velocity = None
            self.path_index = 0

    def _poses(self):
        start = np.asarray(self.config.b_block_starts[self.block], dtype=float)
        goal = np.asarray(self.config.b_block_goals[self.block], dtype=float)
        pick = transform(start + [0, 0, .105], R_DOWN)
        place = transform(goal + [0, 0, .105], R_DOWN)
        pick_hover = pick.copy(); pick_hover[2, 3] += .065
        place_hover = place.copy(); place_hover[2, 3] += .065
        return pick_hover, pick, place_hover, place

    def _start_path(self, goal):
        # Task B uses bounded point-to-point joint motion. Cartesian IK across
        # the table crosses a near-singular strip on this Aloha model.
        self.path, self.velocity, _ = self.kin.joint_path('right', goal)
        self.path_index = 0

    def advance(self, step):
        if self.done:
            return 'task_b_done'
        poses = self._poses()
        # hover-pick, pick, close, lift, hover-place, place, open, retreat
        move_goals = {0: poses[0], 1: poses[1], 3: poses[0], 4: poses[2],
                      5: poses[3], 7: poses[2]}
        if self.phase in move_goals:
            if self.path is None:
                self._start_path(move_goals[self.phase])
            self.kin.drive('right', self.path[self.path_index], self.velocity[self.path_index])
            self.path_index += 1
            if self.path_index >= len(self.path):
                self.phase += 1
                self.path = self.velocity = None
                self.path_index = 0
        else:
            target = 0. if self.phase == 2 else .04
            count = max(2, int(.5 / self.config.dt))
            u = min(1., (self.grip_index + 1) / count)
            self.opening += (target - self.opening) * u
            self.kin.gripper('right', self.opening)
            self.grip_index += 1
            if self.grip_index >= count:
                self.opening = target
                self.grip_index = 0
                self.phase += 1
        if self.phase >= 8:
            self.completed_steps.append(step)
            self.block += 1
            self.phase = 0
        return f'task_b_{self.block}_phase_{self.phase}'


class SharedExecutor:
    """Drive both arm queues before exactly one shared physics step."""

    def __init__(self, env, kin, worker, recorder=None):
        self.env = env
        self.kin = kin
        self.worker = worker
        self.recorder = recorder
        self.events = []

    def tick(self, label, left=None, left_velocity=None, run_b=True, right=None,
             right_velocity=None):
        if left is not None:
            self.kin.drive('left', left, left_velocity)
        if right is not None:
            self.kin.drive('right', right, right_velocity)
        elif run_b:
            self.worker.advance(self.env.steps + 1)
        self.kin.compensate()
        self.env.step_physics()
        changed = not self.events or self.events[-1]['label'] != label
        if changed:
            self.events.append({'label': label, 'start_step': self.env.steps})
        if self.recorder is not None:
            self.recorder.capture(label, force=changed)

    def move(self, arm, label, goal, run_b=True):
        path, velocity, _ = self.kin.cartesian_path(arm, goal)
        for position, speed in zip(path, velocity):
            if arm == 'left':
                self.tick(label, left=position, left_velocity=speed, run_b=run_b)
            else:
                self.tick(label, right=position, right_velocity=speed, run_b=False)

    def move_to_q(self, arm, label, target, run_b=False):
        ids = self.kin.ids[arm]
        start = self.kin.entity.get_qpos()[ids].copy()
        target = np.asarray(target, dtype=float).copy()
        delta = target - start
        duration = max(1.875 * np.max(abs(delta)) / self.env.config.joint_velocity,
                       np.sqrt(5.774 * np.max(abs(delta)) /
                               self.env.config.joint_acceleration), .08)
        count = max(2, int(np.ceil(duration / self.env.config.dt)))
        phase = np.linspace(0., 1., count + 1)
        blend = 10 * phase**3 - 15 * phase**4 + 6 * phase**5
        derivative = (30 * phase**2 - 60 * phase**3 + 30 * phase**4) / duration
        for u, du in zip(blend, derivative):
            position = start + u * delta
            velocity = du * delta
            if arm == 'left':
                self.tick(label, left=position, left_velocity=velocity, run_b=run_b)
            else:
                self.tick(label, right=position, right_velocity=velocity, run_b=False)

    def grip_left(self, label, target):
        start = .04 if target == 0 else 0.
        count = max(2, int(.5 / self.env.config.dt))
        for index in range(count):
            opening = start + (target - start) * (index + 1) / count
            self.kin.gripper('left', opening)
            self.tick(label)

    def helper_round_trip(self, adapter, output, capture):
        ids = self.kin.ids['right']
        previous_q = self.kin.entity.get_qpos()[ids].copy()
        camera_pose = look_at(np.asarray(self.env.config.helper_camera_position),
                              np.asarray(self.env.config.rack_center) + [0, -.03, .02])
        helper_goal = camera_pose @ np.linalg.inv(T_EE_CAMERA)
        helper_q = self.kin.ik('right', helper_goal)[ids]
        self.move_to_q('right', 'helper_move_to_view', helper_q)
        frame = None
        if capture:
            frame = adapter.capture('right', self.env.steps * self.env.config.dt)
            Image.fromarray(frame.rgb).save(output / f'helper_step_{self.env.steps}.png')
        self.move_to_q('right', 'helper_return', previous_q)
        self.worker.invalidate_motion()
        return frame


def _initialise_robot(env, kin, config):
    q = kin.entity.get_qpos().copy()
    for arm, position in [('left', [-.18, -.27, 1.12]), ('right', [.25, -.25, 1.14])]:
        q = kin.ik(arm, transform(position, R_DOWN), q)
        for joint, _, _ in getattr(env.robot, arm + '_gripper'):
            q[kin.entity.get_active_joints().index(joint)] = .04
    kin.entity.set_qpos(q)
    for arm in ('left', 'right'):
        kin.drive(arm, q[kin.ids[arm]])
        kin.gripper(arm, .04)


def _tray_pose_prior(kin, config):
    world_from_ee_at_grasp = transform(
        np.asarray(config.tray_start) + [0, 0, .16], R_DOWN)
    world_from_tray_at_grasp = transform(config.tray_start)
    ee_from_tray = np.linalg.inv(world_from_ee_at_grasp) @ world_from_tray_at_grasp
    return kin.fk('left') @ ee_from_tray


def run_episode(config, seed, policy, b_load, output, record_video=False,
                video_stride=100):
    if policy not in POLICIES:
        raise ValueError(f'unknown policy {policy!r}')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    result = dict(mode='PURE_RGB_ACTIVE_OBSERVATION', observation_modality='rgb_only',
                  eligible_for_strategy_benchmark=True, seed=seed, policy=policy,
                  b_load=b_load, success=False)
    (output / 'config.json').write_text(json.dumps(asdict(config), indent=2))
    env = TrayScene()
    recorder = None
    try:
        env.setup_demo(config, seed, physics_only=False)
        kin = TrayKinematics(env.robot, config)
        _initialise_robot(env, kin, config)
        worker = TaskBWorker(kin, config, b_load)
        if record_video:
            recorder = OverviewRecorder(env, output / 'overview.mp4', video_stride)
            recorder.capture('initial', force=True)
        executor = SharedExecutor(env, kin, worker, recorder)
        adapter = RGBObservationAdapter(env.cameras, env.render)
        estimator = ColorMarkerEstimator(config.rack_center, config.peg_offset,
                                         config.peg_half, config.marker_min_pixels,
                                         config.marker_max_pixels)
        result['initial_qpos_sha256'] = hashlib.sha256(
            kin.entity.get_qpos().tobytes()).hexdigest()

        pre_slot_frames = []
        if policy == 'pre-observation':
            frame = executor.helper_round_trip(adapter, output, capture=True)
            pre_slot_frames.append(frame)

        # Task A uses only nominal/public geometry until RGB correction.
        pick = transform(np.asarray(config.tray_start) + [0, 0, .16], R_DOWN)
        hover = pick.copy(); hover[2, 3] += .06
        executor.move('left', 'pick_hover', hover)
        executor.move('left', 'pick_descend', pick)
        executor.grip_left('close_left_gripper', 0.)
        lift = pick.copy(); lift[2, 3] += .025
        executor.move('left', 'lift_tray', lift)
        partial = lift.copy()
        partial[1, 3] = config.rack_center[1] - config.peg_offset[1] - .04
        partial[2, 3] = config.rack_center[2] + .16
        executor.move('left', 'partial_insertion', partial)

        slot_frames = []
        peg_frames = []
        peg_priors = []
        peg_x_offsets = []
        if policy == 'self-observation':
            previous = kin.fk('left').copy()
            self_goal = previous.copy()
            self_goal[:3, 3] += np.asarray(config.self_view_offset)
            executor.move('left', 'self_move_to_view', self_goal)
            capture_prior = _tray_pose_prior(kin, config)
            frame = adapter.capture('left', env.steps * config.dt)
            Image.fromarray(frame.rgb).save(output / f'self_step_{env.steps}.png')
            slot_frames.append(frame); peg_frames.append(frame)
            peg_priors.append(capture_prior)
            executor.move('left', 'self_return', previous)
            current_prior = _tray_pose_prior(kin, config)
            capture_peg = capture_prior[:3, :3] @ config.peg_offset + capture_prior[:3, 3]
            current_peg = current_prior[:3, :3] @ config.peg_offset + current_prior[:3, 3]
            peg_x_offsets.append(float(current_peg[0] - capture_peg[0]))
        elif policy in ('immediate-helper', 'helper-motion-without-helper-image'):
            frame = executor.helper_round_trip(
                adapter, output, capture=policy == 'immediate-helper')
            if frame is not None:
                slot_frames.append(frame); peg_frames.append(frame)
                peg_priors.append(_tray_pose_prior(kin, config)); peg_x_offsets.append(0.)
        elif policy == 'helper-after-current-B-step':
            target = min(b_load, worker.completed_blocks + 1)
            while worker.completed_blocks < target:
                executor.tick('wait_for_current_task_b_step')
            frame = executor.helper_round_trip(adapter, output, capture=True)
            slot_frames.append(frame); peg_frames.append(frame)
            peg_priors.append(_tray_pose_prior(kin, config)); peg_x_offsets.append(0.)

        if policy in ('no-extra-observation', 'pre-observation',
                      'helper-motion-without-helper-image'):
            frame = adapter.capture('left', env.steps * config.dt)
            Image.fromarray(frame.rgb).save(output / f'normal_step_{env.steps}.png')
            if policy != 'pre-observation':
                slot_frames.append(frame)
            peg_frames.append(frame)
            peg_priors.append(_tray_pose_prior(kin, config)); peg_x_offsets.append(0.)
        if policy == 'pre-observation':
            slot_frames.extend(pre_slot_frames)

        if policy in ('self-observation', 'pre-observation'):
            estimate = estimator.estimate_against_kinematic_peg(
                slot_frames, _tray_pose_prior(kin, config))
        else:
            estimate = estimator.estimate(
                slot_frames, peg_frames, peg_priors, peg_x_offsets)
        result['rgb_estimate'] = estimate.to_dict()
        result['rgb_diagnostics'] = estimator.diagnostics
        if not estimate.valid:
            result['controller_fallback'] = 'zero_correction'
            result['observation_failure'] = estimate.reason
            correction = 0.
        else:
            correction = float(np.clip(
                estimate.lateral_error, -config.max_rgb_correction,
                config.max_rgb_correction))
        corrected = kin.fk('left').copy()
        corrected[0, 3] += correction
        executor.move('left', 'rgb_lateral_correction', corrected)
        final = corrected.copy(); final[1, 3] += config.final_insertion_distance
        executor.move('left', 'final_insertion', final)

        # This is a fixed public schedule, not a GT-triggered early stop. Keep
        # holding A until B finishes, then allow exactly two seconds to settle.
        post_b_steps = 0
        required_post_b = int(config.max_stability_seconds / config.dt)
        while ((not worker.done or post_b_steps < required_post_b)
               and env.steps < 30000):
            label = 'finish_task_b' if not worker.done else 'post_task_b_stability'
            executor.tick(label)
            post_b_steps = post_b_steps + 1 if worker.done else 0
        geometry = measure_geometry(env)
        task_b = measure_task_b(env, b_load)
        task_a_success = bool(geometry['inserted'] and
                              geometry['tray_linear_speed'] < .02)
        a_completion_step = env.steps if task_a_success else None
        success = bool(task_a_success
                       and task_b['task_b_success'])
        result.update(success=success, sim_steps=env.steps,
                      simulation_seconds=env.steps * config.dt,
                      task_a_completion_step=a_completion_step,
                      task_b_completion_steps=worker.completed_steps,
                      final_geometry_gt=geometry, final_task_b_gt=task_b,
                      events=executor.events)
        if recorder is not None:
            recorder.capture('final', force=True)
            result['overview_video'] = 'overview.mp4'
    except Exception as exc:
        result.update(error=f'{type(exc).__name__}: {exc}', sim_steps=env.steps,
                      simulation_seconds=env.steps * config.dt)
        (output / 'error.log').write_text(traceback.format_exc())
        traceback.print_exc()
    finally:
        if recorder is not None:
            recorder.close()
        env.close_env()
        (output / 'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps({key: value for key, value in result.items()
                      if key not in ('events', 'final_geometry_gt', 'final_task_b_gt')}), flush=True)
    return result


def run_batch(config, seeds, loads, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[2]
    rows = []
    for seed in seeds:
        for load in loads:
            for policy in POLICIES:
                run_name = f's{seed}_b{load}_{policy}'
                destination = output / run_name
                command = [sys.executable, str(root / 'scripts/active_observation_tray_demo.py'),
                           'episode', '--policy', policy, '--b-load', str(load),
                           '--seed', str(seed), '--output', str(destination)]
                completed = subprocess.run(command, cwd=root)
                if (destination / 'result.json').exists():
                    row = json.loads((destination / 'result.json').read_text())
                else:
                    row = dict(seed=seed, b_load=load, policy=policy, success=False,
                               error=f'process_exit_{completed.returncode}')
                rows.append(row)
                (output / 'results.json').write_text(json.dumps(rows, indent=2))
                summaries = []
                for item in rows:
                    geometry = item.get('final_geometry_gt', {})
                    task_b = item.get('final_task_b_gt', {})
                    completion = item.get('task_b_completion_steps', [])
                    summaries.append({
                        'seed': item.get('seed'), 'b_load': item.get('b_load'),
                        'policy': item.get('policy'), 'success': item.get('success'),
                        'observation_modality': item.get('observation_modality'),
                        'simulation_seconds': item.get('simulation_seconds'),
                        'sim_steps': item.get('sim_steps'),
                        'task_a_completion_step': item.get('task_a_completion_step'),
                        'task_b_last_completion_step': completion[-1] if completion else None,
                        'lateral_error_gt': geometry.get('lateral_error_gt'),
                        'insertion_error_gt': geometry.get('insertion_error_gt'),
                        'task_b_max_error_gt': max(task_b.get('task_b_errors_gt', []),
                                                   default=None),
                        'error': item.get('error'),
                    })
                with (output / 'results.csv').open('w', newline='') as stream:
                    writer = csv.DictWriter(stream, fieldnames=summaries[0].keys())
                    writer.writeheader(); writer.writerows(summaries)
    return rows
