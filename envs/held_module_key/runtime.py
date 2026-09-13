"""Small single-route runner. Privileged state only enters audit/evaluation files."""
import hashlib
import json
import sys
import time
import traceback
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation
from envs.occluded_socket.geometry import T_EE_MODULE,T_EE_CAMERA,transform
from envs.occluded_socket.observation import ObservationAdapter
from envs.occluded_socket.recording import Recorder,save_json,state_hash
from .scene import HeldKeyScene,KeyKinematics,initialize
from .controller import KeyController,rank_routes,rank_joint_presentations
from .perception import KeyPerception

ROOT=Path(__file__).resolve().parents[2]


def mounting_audit():
    urdf=ROOT/'assets/embodiments/aloha-agilex/urdf/arx5_description_isaac.urdf'
    root=ET.parse(urdf).getroot()
    rows=[]
    for joint in root.findall('joint'):
        if joint.attrib['name'] in ['left_camera_joint','right_camera_joint','fl_joint6','fr_joint6','fl_joint7','fl_joint8','fr_joint7','fr_joint8']:
            rows.append(dict(name=joint.attrib['name'],type=joint.attrib['type'],
                             **{x.tag:x.attrib for x in joint}))
    return dict(urdf=str(urdf),sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),joints=rows,
                conclusion='Cameras are fixed to link6. Fingers have prismatic joints relative to link6. Camera-to-contact/object rigidity requires stable jaw opening and non-slipping grasp; it is measured, never assumed exact.')


def run(args,config):
    out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    version={}
    paths=[*sorted((ROOT/'envs/held_module_key').glob('*.py')),ROOT/'scripts/held_module_key_demo.py',
           ROOT/'env_cfg/task_config/held_module_key.yml',ROOT/'envs/occluded_socket_drawer.py',
           *sorted((ROOT/'envs/occluded_socket').glob('*.py'))]
    for p in paths:
        name=str(p.relative_to(ROOT));version[name]=hashlib.sha256(p.read_bytes()).hexdigest()
        dest=out/'source'/name;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(p.read_bytes())
    config_id='held_key_'+state_hash(config)[:12]
    result=dict(run_id=out.name,instance_id=f'{config_id}/development_{args.seed}',config_id=config_id,
                policy_id=args.route,seed=args.seed,mode='planning' if args.plan else 'install' if args.install else 'geometry',
                initialized=False,attempt_completed=False,observation_valid=False,success=False,failure_reason=None,
                t_information=None,t_install_ready=None,t_install_ready_physical=None,t_right_stable=None,t_recovered=None)
    result.update(scene_source_id=version['envs/held_module_key/scene.py'][:12],
                  policy_source_id=version['envs/held_module_key/controller.py'][:12],
                  diagnostic_cycle=bool(args.cycle),candidates_from=str(args.candidates_from) if args.candidates_from else None)
    save_json(out/'provenance.json',dict(source_sha256=version,command=sys.argv,gpu=args.gpu,development_only=True))
    save_json(out/'config.json',config);save_json(out/'mounting_chain.json',mounting_audit())
    env=None;rec=None;start=time.perf_counter();events=[];poses=[];done_since=None;stable_since=None;grasp_since=None
    try:
        env=HeldKeyScene();env.setup_demo(seed=args.seed,config=config,gpu=args.gpu,include_drawer=False)
        kin=KeyKinematics(env.robot,config)
        initialization=initialize(env,kin)
        save_json(out/'initialization_gt.json',initialization)
        save_json(out/'initial_state_gt.json',env.snapshot())
        if not initialization['valid']:
            env.render()
            for name,cam in env.camera_map.items():
                cam.take_picture();Image.fromarray((cam.get_picture('Color')[...,:3]*255).clip(0,255).astype('uint8')).save(out/f'initialization_failure_{name}.png')
            raise RuntimeError('initialization_grasp_lost')
        result['initialized']=True
        save_json(out/'initial_state_gt.json',env.snapshot())
        result['initial_state_sha256']=state_hash(env.snapshot())
        env.render()
        initial_adapter=ObservationAdapter({a:env.camera_map[a] for a in ['left','right']},env.camera_map['head'],kin)
        initial_frames=initial_adapter.capture(0.)
        initial_dir=out/'frames'/'initial';initial_dir.mkdir(parents=True,exist_ok=True)
        for name,f in initial_frames.items():
            Image.fromarray(f.rgb).save(initial_dir/f'{name}.png')
        save_json(initial_dir/'robot_camera.json',dict(timestamp=0.,qpos=kin.entity.get_qpos(),qvel=kin.entity.get_qvel(),
                  cameras={n:dict(T_world_camera=f.T_world_camera,intrinsic=f.intrinsic) for n,f in initial_frames.items()}))
        # An explicit allow-list: no seed, random ranges, hidden key, actor, or scene.
        public={k:config[k] for k in ['dt','socket_center','module_half_size','rack_center']}
        if args.candidates_from:
            all_candidates=json.loads(args.candidates_from.read_text())
            feasible=sorted([r for r in all_candidates if r['feasible']],key=lambda r:r['seconds'])
            for r in feasible:r['goal']=np.array(r['goal'])
        elif args.joint_search:
            feasible,all_candidates=rank_joint_presentations(args.route,kin,public)
        else:
            feasible,all_candidates=rank_routes(args.route,kin,public)
        save_json(out/'candidate_plans.json',all_candidates)
        if not feasible:raise RuntimeError('no_feasible_robot_path_in_bounded_candidates')
        candidate=feasible[args.pick]
        save_json(out/'selected_candidate.json',candidate)
        if args.plan:
            result['attempt_completed']=True
            return result
        rec=Recorder(out,config,True)
        def event(row):
            events.append(row);rec.log('controller',row);rec.files['controller'].flush()
        controller=KeyController(kin,public,event,args.route,candidate,args.install,args.cycle)
        adapter=ObservationAdapter({a:env.camera_map[a] for a in ['left','right']},env.camera_map['head'],kin)
        perception=KeyPerception(config['key_marker_points'],config['perception'])
        def snapshot(label,frames):
            target=out/'frames'/label;target.mkdir(parents=True,exist_ok=True)
            for name,f in frames.items():
                Image.fromarray(f.rgb).save(target/f'{name}.png')
                np.savez_compressed(target/f'{name}.npz',rgb=f.rgb,depth=f.depth,intrinsic=f.intrinsic,
                                    T_world_camera=f.T_world_camera,timestamp=f.timestamp)
            save_json(target/'robot_camera.json',dict(timestamp=env.sim_time,
                      qpos=kin.entity.get_qpos(),qvel=kin.entity.get_qvel(),
                      cameras={n:dict(T_world_camera=f.T_world_camera,intrinsic=f.intrinsic) for n,f in frames.items()}))
        initial_relative=np.linalg.inv(kin.fk('right'))@env.module.get_pose().to_transformation_matrix()
        env.decision_started=True
        frames=None;saved_first=False;last_phase=None
        period=round(1/config['camera_hz']/config['dt'])
        grip_joints=[kin.entity.get_active_joints().index(j[0]) for j in env.robot.right_gripper]
        for tick in range(round(config['budget']/config['dt'])+1):
            t=env.sim_time
            if tick%period==0:
                env.render();frames=adapter.capture(t)
                detections=[perception.detect(f,kin.fk('right'),controller.held) for f in frames.values()]
                for d in detections:rec.log('perception',d)
                controller.observe(t,frames,detections)
                if tick==0:
                    snapshot('initial',frames)
                    errors={}
                    for name,f in frames.items():
                        ext=np.eye(4);ext[:3]=env.camera_map[name].get_extrinsic_matrix()
                        errors[name]=float(np.max(abs(f.T_world_camera-np.linalg.inv(ext))))
                    save_json(out/'rendered_camera_vs_fk.json',errors)
                if controller.information is not None and not saved_first:
                    saved_first=True;snapshot('first_information',frames)
                if controller.phase!=last_phase:
                    snapshot(f'{t:.1f}_{controller.phase}',frames);last_phase=controller.phase
                rec.video(env,frames,args.route,'geometry' if not args.install else 'installation',controller.status,
                          'valid' if controller.information is not None else 'searching')
            T_module=env.module.get_pose().to_transformation_matrix()
            T_relative=np.linalg.inv(kin.fk('right'))@T_module
            displacement=float(np.linalg.norm(T_relative[:3,3]-initial_relative[:3,3]))
            rotation=float(np.rad2deg(Rotation.from_matrix(T_relative[:3,:3]@initial_relative[:3,:3].T).magnitude()))
            contacts=[]
            for contact in env.scene.get_contacts():
                names=[b.entity.name for b in contact.bodies]
                impulse=sum(np.linalg.norm(p.impulse) for p in contact.points)
                if impulse>.00001 and ('key_module' in names or any(n.startswith(('fl_link','fr_link')) for n in names)):
                    contacts.append(dict(bodies=names,impulse=float(impulse)))
            T_key=T_module@env.T_module_key
            error=T_key[:3,3]-np.array(config['socket_center'])
            angle=float(np.rad2deg(Rotation.from_matrix(T_key[:3,:3]).magnitude()))
            socket_contact=any('key_module' in c['bodies'] and 'key_socket' in c['bodies'] for c in contacts)
            grasp_contact=all(any('key_module' in c['bodies'] and finger in c['bodies'] for c in contacts) for finger in ['fr_link7','fr_link8'])
            grasp_since=(t if grasp_since is None else grasp_since) if grasp_contact else None
            if hasattr(controller,'pre_goal') and result['t_install_ready_physical'] is None and controller.status['right'] in ['install_preapproach','install_preapproach_done']:
                ee=kin.fk('right');goal=controller.pre_goal
                pe=float(np.linalg.norm(ee[:3,3]-goal[:3,3]));ae=float(Rotation.from_matrix(ee[:3,:3]@goal[:3,:3].T).magnitude())
                speed=float(np.max(abs(kin.entity.get_qvel()[kin.ids['right']])))
                if pe<.002 and ae<np.deg2rad(1) and speed<.05 and grasp_contact:
                    result['t_install_ready_physical']=t
                    event(dict(timestamp=t,event='installation_ready_physical',translation_error=pe,angle_error_radians=ae,max_joint_speed=speed))
            e=config['evaluation']
            seated=(controller.right_released and socket_contact and np.linalg.norm(error[:2])<e['xy'] and abs(error[2])<e['z'] and angle<e['angle_degrees'] and np.linalg.norm(env.module_body.linear_velocity)<e['linear_speed'] and np.linalg.norm(env.module_body.angular_velocity)<e['angular_speed'])
            stable_since=(t if stable_since is None else stable_since) if seated else None
            stable=stable_since is not None and t-stable_since>=e['stable_seconds']
            if tick%5==0:
                row=dict(timestamp=t,held_command=controller.held,phase=controller.phase,T_ee_module=T_relative,
                         translation_drift=displacement,rotation_drift_degrees=rotation,
                         T_camera_module={a:np.linalg.inv(getattr(env.robot,a+'_camera').get_pose().to_transformation_matrix())@T_module for a in ['left','right']},
                         jaw_qpos=kin.entity.get_qpos()[grip_joints],module_linear_velocity=env.module_body.linear_velocity,
                         module_angular_velocity=env.module_body.angular_velocity,T_world_key=T_key,
                         key_position_error=error,key_angle_error_degrees=angle,seated=bool(seated),stable=bool(stable),both_jaw_contact=grasp_contact)
                poses.append(row);rec.log('evaluator_gt',row)
                rec.log('contacts_gt',dict(timestamp=t,contacts=contacts))
                rec.log('joints',dict(timestamp=t,qpos=kin.entity.get_qpos(),qvel=kin.entity.get_qvel()))
            if stable and result['t_right_stable'] is None:result['t_right_stable']=t
            if controller.phase=='done':
                if done_since is None:done_since=t
                if t-done_since>=1.0:
                    result['attempt_completed']=True
                    result['success']=bool(stable) if args.install else controller.information is not None and grasp_since is not None and t-grasp_since>=.3
                    break
            if controller.failure:
                result['failure_reason']=controller.failure;break
            controller.tick(t)
            env.step_physics()  # Sole physical step after t=0; no post-init teleports.
        env.render();frames=adapter.capture(env.sim_time)
        snapshot('final',frames)
        save_json(out/'final_state_gt.json',env.snapshot())
        def first(event_name,action=None):
            return next((x['timestamp'] for x in events if x['event']==event_name and (action is None or x.get('name')==action)),None)
        result.update(observation_valid=controller.information is not None,t_information=controller.t_information_ready,
                      information_camera=next((x['camera'] for x in events if x['event']=='information_ready'),None),
                      t_install_ready=first('action_end','install_preapproach'),
                      t_left_restored=first('action_end','helper_restore'),t_recovered_command=first('recovery_complete'),
                      t_recovered=first('recovery_complete') if args.install or (grasp_since is not None and env.sim_time-grasp_since>=.3) else None,
                      final_both_jaw_contact=bool(grasp_since is not None and env.sim_time-grasp_since>=.3),
                      t_right_exit=first('action_end','install_retreat'),physics_seconds=env.sim_time,
                      planning_wall_seconds=controller.planning_wall)
        held=[r for r in poses if r['held_command'] and r['phase'] not in ['placing','regrasp'] and not controller.right_released]
        # Release episodes still need pre-release grasp auditing.
        release=first('action_end','install_release')
        if release is not None:held=[r for r in poses if r['held_command'] and r['timestamp']<release and r['phase'] not in ['placing','regrasp']]
        result['held_drift_gt']=dict(samples=len(held),translation_max=max((r['translation_drift'] for r in held),default=None),
                                    rotation_max_degrees=max((r['rotation_drift_degrees'] for r in held),default=None))
        if not result['success'] and result['failure_reason'] is None:
            result['failure_reason']='no_valid_visual_key' if controller.information is None else 'not_stably_seated' if args.install else 'route_not_completed'
    except Exception as exc:
        result['failure_reason']=f'{type(exc).__name__}: {exc}'
        (out/'error.log').write_text(traceback.format_exc());traceback.print_exc()
    finally:
        if rec:rec.close()
        if env:env.close_env()
        result['wall_seconds']=time.perf_counter()-start
        save_json(out/'result.json',result)
        print(json.dumps(result),flush=True)
    return result
