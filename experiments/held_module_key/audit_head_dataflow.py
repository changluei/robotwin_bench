"""Offline audit of the saved S image-to-install-target chain; no simulation/GT files."""
import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
RUN = ROOT / 'result/held_module_key/final400_S'
OUT = ROOT / 'experiments/held_module_key/head_dataflow_audit'


def module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def main():
    OUT.mkdir(parents=True, exist_ok=False)
    files = [RUN / 'config.json', RUN / 'perception.jsonl', RUN / 'controller.jsonl',
             RUN / 'frames/first_information/head.npz', RUN / 'frames/first_information/right.npz',
             RUN / 'source/envs/held_module_key/perception.py', RUN / 'source/envs/occluded_socket/geometry.py']
    config = json.loads(files[0].read_text())
    perception_log = [json.loads(line) for line in files[1].read_text().splitlines()]
    controller_log = [json.loads(line) for line in files[2].read_text().splitlines()]
    vision = module(files[5], 'frozen_held_key_vision')
    geo = module(files[6], 'frozen_held_key_geometry')
    head = np.load(files[3]); right = np.load(files[4])
    # Public robot FK recovered from the recorded wrist extrinsic and URDF mount.
    # No actor pose, key pose, scene generator, instance seed or evaluator is read.
    T_world_ee = right['T_world_camera'] @ np.linalg.inv(geo.T_EE_CAMERA @ geo.T_SAPIEN_CV)
    frame = dict(name='head', timestamp=float(head['timestamp']), depth=head['depth'],
                 intrinsic=head['intrinsic'], T_world_camera=head['T_world_camera'])
    rgb = head['rgb']

    def detect(input_rgb, input_depth=None):
        f = dict(frame)
        if input_depth is not None: f['depth'] = input_depth
        return vision.KeyPerception(config['key_marker_points'], config['perception']).detect(
            SimpleNamespace(rgb=input_rgb, **f), T_world_ee)

    original = detect(rgb)
    logged = next(d for d in perception_log if d['camera']=='head' and d['valid'])
    target = next(d for d in controller_log if d['event']=='installation_target')
    predicted_target = geo.transform(config['socket_center']) @ np.linalg.inv(original['T_ee_key'])
    predicted_target[2,3] += .0005
    # Synthetic input ablations are offline checks, never physical H/S results.
    no_rgb = detect(np.zeros_like(rgb))
    no_depth = detect(rgb, np.zeros_like(frame['depth']))
    h,s,v = cv2.split(cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV))
    masks = [(h<9)|(h>172), (h>42)&(h<82), (h>102)&(h<132), (h>20)&(h<38)]
    relabeled = rgb.copy()
    for mask, color in zip(masks, [(0,0,255),(255,255,0),(255,0,0),(0,255,0)]):
        relabeled[mask & (s>130) & (v>55)] = color
    changed = detect(relabeled)
    changed_target = geo.transform(config['socket_center']) @ np.linalg.inv(changed['T_ee_key'])
    angle_change = np.rad2deg(Rotation.from_matrix(changed_target[:3,:3] @ predicted_target[:3,:3].T).magnitude())
    pose_error = float(np.max(abs(original['T_ee_key'] - np.array(logged['T_ee_key']))))
    target_error = float(np.max(abs(predicted_target - np.array(target['T_world_ee']))))
    assert pose_error < 1e-5 and target_error < 1e-5
    assert 'T_ee_key' not in no_rgb and 'T_ee_key' not in no_depth
    assert angle_change > 179
    result = dict(
        source_run=str(RUN), mode='offline_saved_observation_replay_no_simulation',
        inputs_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        gt_files_read=[], timestamp=original['timestamp'], camera='head',
        measured_pixels=original['pixels'].tolist(), measured_counts=[int(x) for x in original['counts']],
        geometry_error_metres=float(original['geometry_error']), reprojection_error_pixels=float(original['reprojection_error']),
        pose_matrix_max_difference_from_recorded_estimate=pose_error,
        installation_target_matrix_max_difference_from_recorded_command=target_error,
        black_rgb=dict(reason=no_rgb['reason'],produces_pose='T_ee_key' in no_rgb),
        zero_depth=dict(reason=no_depth['reason'],produces_pose='T_ee_key' in no_depth),
        swapped_color_correspondence=dict(installation_orientation_change_degrees=float(angle_change),
                                         synthetic_offline_input=True),
        temporal_validation='Single saved RGB-D frame recomputes pose only; two-frame acceptance is evidenced by original 2.5/2.6 s logs, not replayed from two saved raw frames.',
        public_priors=['Fixed color-to-key template coordinates','RGB-D intrinsics and camera extrinsics','Robot FK','Fixed socket frame'],
        limitation='Marker-dependent ideal RGB-D pose estimation; no independent unmarked key-shape recognition or realistic depth-noise robustness demonstrated.')
    (OUT / 'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__ == '__main__': main()
