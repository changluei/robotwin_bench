"""Development only: initialize calibrated checkpoint, test fixed candidate views."""
import argparse
import json
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
from PIL import Image
from envs.occluded_socket_drawer import occluded_socket_drawer
from envs.occluded_socket.motion import RobotKinematics
from envs.occluded_socket.geometry import T_EE_MODULE, T_EE_CAMERA, look_at
from envs.occluded_socket.observation import ObservationAdapter
from envs.occluded_socket.perception import Perception
from envs.occluded_socket.runtime import initialize, code_version
from envs.occluded_socket.recording import save_json


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--gpu', type=int, default=0)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    env = occluded_socket_drawer()
    env.setup_demo(gpu=args.gpu, include_drawer=False)
    kin = RobotKinematics(env.robot, env.config)
    initialization=initialize(env,kin)
    save_json(args.output/'initialization_gt.json',initialization)
    save_json(args.output/'provenance.json',code_version())
    save_json(args.output/'config.json',env.config)
    env.render()
    adapter = ObservationAdapter({a:env.camera_map[a] for a in ['left','right']},env.camera_map['head'],kin)
    perception = Perception(env.config['marker_points'],env.config['perception'])
    env.decision_started=True
    records=[]
    def save(label):
        for _ in range(2):
            env.render()
            frames=adapter.capture(env.sim_time)
            for a,f in frames.items():
                d=perception.detect(f)
                print('detection',label,a,d.to_dict(),flush=True)
                extrinsic=np.eye(4);extrinsic[:3]=env.camera_map[a].get_extrinsic_matrix()
                error=np.max(abs(f.T_world_camera-np.linalg.inv(extrinsic)))
                relative=np.linalg.inv(kin.fk('right'))@env.module.get_pose().to_transformation_matrix()
                records.append(dict(label=label,**d.to_dict(),camera_matrix_error=float(error),
                               held_module_translation_error_gt=float(np.linalg.norm(relative[:3,3]-T_EE_MODULE[:3,3]))))
            for _ in range(25):
                kin.compensate();env.step_physics()
        for name, cam in env.camera_map.items():
            cam.take_picture()
            pixels=(cam.get_picture('Color')[...,:3]*255).clip(0,255).astype('uint8')
            Image.fromarray(pixels).save(args.output/f'{label}_{name}.png')
        print(label,'fk', {a:kin.fk(a).tolist() for a in ['left','right']},flush=True)
    save('checkpoint')
    candidates={'left': [[0,-.22,1.04]],
                'right': [[.35,-.28,1.07],[.35,-.28,1.07]]}
    for arm, positions in candidates.items():
        for i, pos in enumerate(positions):
            try:
                camera=look_at(np.array(pos),env.config['socket_center'])
                if arm=='right':
                    aim=np.array(env.config['socket_center'])-camera[:3,1]*(.10 if i%2==0 else .16)
                    camera=look_at(np.array(pos),aim)
                goal=camera@np.linalg.inv(T_EE_CAMERA)
                print('candidate',arm,i,'goal',goal.tolist(),flush=True)
                path,vel,wall=kin.joint_path(arm,goal)
                collisions=kin.check_path(arm,path)
                if collisions:
                    raise RuntimeError(f'probe path collision: {collisions}')
                for qp,qv in zip(path,vel):
                    kin.compensate();kin.drive(arm,qp,qv);env.step_physics()
                for _ in range(80):
                    kin.compensate();env.step_physics()
                save(f'{arm}_{i}')
            except RuntimeError as e:
                print(str(e),flush=True)
    save_json(args.output/'snapshot_gt.json',env.snapshot())
    save_json(args.output/'detections_and_calibration.json',records)

if __name__=='__main__':
    main()
