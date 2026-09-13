"""Privileged experiment harness. Keeps evaluation and policies separate."""
import copy
import hashlib
import importlib.metadata
import json
import platform
import subprocess
import sys
import time
import traceback
from pathlib import Path
import numpy as np
import sapien
from PIL import Image
from envs.occluded_socket_drawer import occluded_socket_drawer
from .geometry import transform, R_DOWN, T_EE_MODULE
from .motion import RobotKinematics
from .observation import ObservationAdapter
from .perception import Perception
from .controller import Controller
from .evaluation import Evaluator
from .recording import Recorder, save_json, state_hash

ROOT=Path(__file__).resolve().parents[2]


def code_version():
    files=[ROOT/'envs/occluded_socket_drawer.py',*sorted((ROOT/'envs/occluded_socket').glob('*.py')),
           ROOT/'env_cfg/task_config/occluded_socket_drawer.yml', ROOT/'scripts/occluded_socket_demo.py']
    return dict(commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
                status=subprocess.check_output(['git','status','--short'],cwd=ROOT,text=True),
                source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files if p.exists()})


def initialize(env,kin):
    """Explicit cold checkpoint; contact settling is outside the decision clock.

    No observation history is created before this checkpoint. All four groups
    use identical right-side initialization, independent of seed and condition.
    """
    q=kin.entity.get_qpos()
    goals={'right':transform([.18,-.13,1.10],R_DOWN),'left':transform([-.25,-.25,1.14],R_DOWN)}
    for arm,goal in goals.items():
        q=kin.ik(arm,goal,q)
    for arm in ['left','right']:
        ids=[kin.entity.get_active_joints().index(j[0]) for j in getattr(env.robot,arm+'_gripper')]
        q[ids]=.016 if arm=='right' else 0
    kin.entity.set_qpos(q)
    for arm in goals:
        kin.drive(arm,q[kin.ids[arm]])
        kin.gripper(arm,0)
    env.module.set_pose(sapien.Pose(goals['right']@T_EE_MODULE))
    for _ in range(750):
        kin.compensate()
        env.step_physics()
    # Initialization validation only; never used as policy calibration.
    relative=np.linalg.inv(kin.fk('right'))@env.module.get_pose().to_transformation_matrix()
    if np.linalg.norm(relative[:3,3]-T_EE_MODULE[:3,3])>.025:
        raise RuntimeError('checkpoint_grasp_unstable')
    kin.reference_qpos=kin.entity.get_qpos().copy()
    return dict(cold_start=True,initialization_physics_time=750*env.config['dt'],
                designed_T_ee_module=T_EE_MODULE,measured_T_ee_module_evaluation_only=relative)


def run_episode(args,config):
    output=Path(args.output)
    output.mkdir(parents=True,exist_ok=False)
    result=dict(run_id=output.name,instance_id=f'seed_{args.seed}',seed=args.seed,policy=args.policy,variant=args.variant,
                load_condition=args.load_condition,mode=args.mode,experiment='cold_checkpoint_decision',
                success=False,timeout=False,failure_reason=None,t_information_ready=None,
                t_right_complete=None,t_left_complete=None,t_all_complete=None)
    save_json(output/'config.json',config)
    version=code_version()
    save_json(output/'provenance.json',dict(**version,command=[sys.executable,*sys.argv],gpu=args.gpu,
                python=sys.version,platform=platform.platform(),versions={p:importlib.metadata.version(p) for p in
                ['sapien','mplib','numpy','opencv-python','torch','scipy','imageio']},
                timing='sum(dt); planning/rendering wall time separate; cold checkpoint has no prior observations'))
    # Preserve exact task source, including uncommitted experiment changes.
    for file in version['source_sha256']:
        dest=output/'source'/file
        dest.parent.mkdir(parents=True,exist_ok=True)
        dest.write_bytes((ROOT/file).read_bytes())
    env=None;rec=None
    started=time.perf_counter()
    try:
        env=occluded_socket_drawer()
        env.setup_demo(seed=args.seed,load_condition=args.load_condition,config=config,gpu=args.gpu,
                       headless=args.headless,include_drawer=not args.right_only)
        kin=RobotKinematics(env.robot,config)
        init=initialize(env,kin)
        save_json(output/'initialization_gt.json',init)
        initial=env.snapshot()
        result['initial_state_sha256']=state_hash(initial)
        save_json(output/'initial_state_gt.json',initial)
        env.render()
        adapter=ObservationAdapter({a:env.camera_map[a] for a in ['left','right']},env.camera_map['head'],kin)
        perception=Perception(config['marker_points'],config['perception'])
        rec=Recorder(output,config,args.record)
        # Explicit public schema: no load label, seed or random range in policy config.
        public={k:copy.deepcopy(config[k]) for k in ['dt','socket_center','module_half_size','drawer_center']}
        controller=Controller(args.policy,kin,public,lambda row:rec.log('controller',row),args.variant)
        evaluator=Evaluator(env)
        if args.mode=='drawer':
            controller.policy='S'
            controller.observation_enabled=False
        if args.right_only:
            controller.drawer_started=True
            controller.status['left']='no_drawer_stage1'
        if args.mode=='oracle':
            # ORACLE-ONLY: privileged target injection. No fallback from visual mode.
            controller.information=env.socket.get_pose().to_transformation_matrix()
            controller.t_information_ready=0.
            controller.event(0,'ORACLE_ONLY_target_injection',T_world_socket=controller.information)
        env.decision_started=True
        frames=None;detections=[]
        frame_period=round(1/config['camera_hz']/config['dt'])
        video_period=round(1/config['video_fps']/config['dt'])
        previous_q=kin.entity.get_qpos().copy()
        both_moving_ticks=0
        right_motion_ticks=0;left_motion_ticks=0
        terminal=False
        for tick in range(round(config['budget']/config['dt'])+1):
            t=env.sim_time
            if tick%frame_period==0:
                env.render()
                frames=adapter.capture(t)
                detections=[perception.detect(f) for f in frames.values()]
                for d in detections:
                    rec.log('perception',d.to_dict())
                    error=evaluator.estimate_error(d)
                    if error:
                        rec.log('evaluator_gt',dict(event='localization_error',**error))
                if args.mode not in ['oracle','drawer']:
                    controller.observe(t,frames,detections)
                else:
                    controller.observe(t,frames,[])
                if tick==0:
                    for name,f in frames.items():
                        Image.fromarray(f.rgb).save(output/f'checkpoint_{name}.png')
                    calibration={}
                    for name,f in frames.items():
                        ext=np.eye(4);ext[:3]=env.camera_map[name].get_extrinsic_matrix()
                        calibration[name]=dict(intrinsic=f.intrinsic,T_world_camera=f.T_world_camera,
                                              matrix_max_error=float(np.max(abs(f.T_world_camera-np.linalg.inv(ext)))))
                    save_json(output/'camera_calibration.json',calibration)
                for d in detections:
                    if d.valid and not (output/f'valid_{d.camera}.png').exists():
                        frame=frames[d.camera]
                        Image.fromarray(frame.rgb).save(output/f'valid_{d.camera}.png')
                        np.savez_compressed(output/f'valid_{d.camera}_observation.npz',rgb=frame.rgb,depth=frame.depth,
                                            intrinsic=frame.intrinsic,T_world_camera=frame.T_world_camera,timestamp=frame.timestamp)
            if tick%video_period==0:
                rec.video(env,frames,args.policy,args.load_condition,controller.status,
                          'ready' if controller.information is not None else 'searching')
            state=evaluator.sample(t,controller.right_released)
            if tick%5==0:
                rec.log('evaluator_gt',state)
            all_idle=all(controller.active[a] is None and not controller.queues[a] for a in ['left','right'])
            task_done=(evaluator.current['left'] and controller.drawer_started) if args.mode=='drawer' else (evaluator.current['all'] and controller.install_started)
            if task_done and all_idle and controller.failure is None:
                result['success']=True
                result['t_all_complete']=t
                if tick%video_period:
                    env.render()
                    final_frames=adapter.capture(t)
                    rec.video(env,final_frames,args.policy,args.load_condition,controller.status,'complete')
                terminal=True
                break
            if t>=config['budget']:
                break
            controller.tick(t)
            env.step_physics()  # THE ONLY post-checkpoint physical step in the harness.
            q=kin.entity.get_qpos().copy()
            moving={a:bool(np.linalg.norm((q-previous_q)[kin.ids[a]])/config['dt']>.025) for a in ['left','right']}
            both_moving_ticks+=moving['left'] and moving['right']
            left_motion_ticks+=moving['left'];right_motion_ticks+=moving['right']
            previous_q=q
            if tick%5==0:
                rec.log('joints',dict(timestamp=env.sim_time,qpos=q,qvel=kin.entity.get_qvel(),moving=moving,
                                     left_state=controller.status['left'],right_state=controller.status['right']))
                contacts=[]
                for contact in env.scene.get_contacts():
                    names=[b.entity.name for b in contact.bodies]
                    impulse=sum(np.linalg.norm(p.impulse) for p in contact.points)
                    if impulse>.0001 and (any(n in names for n in ['module','drawer_slide','fixed_housing','hidden_socket']) or any(n.startswith(('fl_link','fr_link')) for n in names)):
                        contacts.append(dict(bodies=names,impulse=float(impulse)))
                if contacts:
                    rec.log('contacts_gt',dict(timestamp=env.sim_time,contacts=contacts))
        result.update(t_information_ready=controller.t_information_ready,terminal_physics_time=env.sim_time,
                      t_right_complete=evaluator.times['right'] if evaluator.current['right'] else None,
                      t_left_complete=evaluator.times['left'] if evaluator.current['left'] else None,
                      timeout=not terminal,planning_wall_seconds=controller.planning_wall,
                      both_arms_moving_seconds=both_moving_ticks*config['dt'],
                      left_moving_seconds=left_motion_ticks*config['dt'],right_moving_seconds=right_motion_ticks*config['dt'])
        if not terminal:
            result['failure_reason']=controller.failure or ('drawer_not_closed' if args.mode=='drawer' else 'perception_timeout' if controller.information is None else
                        'right_not_seated' if not evaluator.current['right'] else 'drawer_not_closed')
        save_json(output/'final_state_gt.json',env.snapshot())
    except Exception as e:
        result['failure_reason']=f'{type(e).__name__}: {e}'
        (output/'error.log').write_text(traceback.format_exc(),encoding='utf-8')
        traceback.print_exc()
    finally:
        if rec:
            rec.close()
        if env:
            env.close_env()
        result['wall_seconds']=time.perf_counter()-started
        save_json(output/'result.json',result)
        print(json.dumps(result),flush=True)
    return result
