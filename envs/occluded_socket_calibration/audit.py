"""Passive instrumentation of the unchanged v1 physics/control loop."""
import hashlib
import json
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation
from envs.occluded_socket import runtime
from envs.occluded_socket.observation import ObservationAdapter
from envs.occluded_socket.recording import Recorder, save_json

ROOT=Path(__file__).resolve().parents[2]


def instrumented_episode(args, config):
    source_paths=[*sorted(Path(__file__).parent.glob('*.py')),ROOT/'scripts/calibrate_occluded_socket.py',
                  *sorted((ROOT/'env_cfg/task_config').glob('occluded_socket_drawer_v2*.yml'))]
    source_bytes={str(p.relative_to(ROOT)):p.read_bytes() for p in source_paths}
    context={}
    original_initialize=runtime.initialize
    original_recorder=runtime.Recorder

    def initialize(env,kin):
        value=original_initialize(env,kin)
        context.update(env=env,kin=kin,motion={a:dict(joint_path_rad=0.,ee_path_metres=0.,
                                                   ee_rotation_rad=0.) for a in ['left','right']})
        previous_q=kin.entity.get_qpos().copy()
        previous_T={a:kin.fk(a).copy() for a in ['left','right']}
        step=env.step_physics

        def measured_step():
            nonlocal previous_q,previous_T
            step()
            q=kin.entity.get_qpos().copy()
            for arm in ['left','right']:
                T=kin.fk(arm).copy();m=context['motion'][arm]
                m['joint_path_rad']+=float(np.sum(abs(q[kin.ids[arm]]-previous_q[kin.ids[arm]])))
                m['ee_path_metres']+=float(np.linalg.norm(T[:3,3]-previous_T[arm][:3,3]))
                m['ee_rotation_rad']+=float(Rotation.from_matrix(previous_T[arm][:3,:3].T@T[:3,:3]).magnitude())
                previous_T[arm]=T
            previous_q=q
        env.step_physics=measured_step
        return value

    class AuditRecorder(original_recorder):
        def __init__(self,*values):
            super().__init__(*values)
            self.milestones=[]

        def state(self,t):
            kin=context['kin']
            return dict(timestamp=t,qpos=kin.entity.get_qpos(),qvel=kin.entity.get_qvel(),
                        T_world_ee={a:kin.fk(a) for a in ['left','right']},
                        motion_before_event=json.loads(json.dumps(context['motion'])))

        def capture_event(self,label,t):
            out=self.output/'audit_frames'/label
            out.mkdir(parents=True,exist_ok=False)
            env,kin=context['env'],context['kin']
            adapter=ObservationAdapter({a:env.camera_map[a] for a in ['left','right']},env.camera_map['head'],kin)
            frames=adapter.capture(t)
            cameras={}
            for name,frame in frames.items():
                Image.fromarray(frame.rgb).save(out/f'{name}.png')
                np.savez_compressed(out/f'{name}.npz',rgb=frame.rgb,depth=frame.depth,
                                    intrinsic=frame.intrinsic,T_world_camera=frame.T_world_camera,timestamp=t)
                cameras[name]=dict(intrinsic=frame.intrinsic,T_world_camera=frame.T_world_camera)
            save_json(out/'state.json',dict(**self.state(t),cameras=cameras))

        def log(self,stream,row):
            super().log(stream,row)
            if stream=='controller' and row['event'] in ['action_start','action_end','action_cancel',
                    'information_ready','installation_target','drawer_plan','planning_failure','avoidance_replan']:
                self.milestones.append(dict(**self.state(row['timestamp']),event=row))
                if row['event']=='information_ready':
                    self.capture_event('first_information',row['timestamp'])

        def video(self,env,frames,*values):
            if env.sim_time==0:
                self.capture_event('initial',0.)
            super().video(env,frames,*values)

        def close(self):
            save_json(self.output/'audit_milestones.json',self.milestones)
            super().close()

    runtime.initialize=initialize
    runtime.Recorder=AuditRecorder
    try:
        result=runtime.run_episode(args,config)
    finally:
        runtime.initialize=original_initialize
        runtime.Recorder=original_recorder
    path=Path(args.output)
    events=[json.loads(line) for line in (path/'controller.jsonl').read_text().splitlines()] if (path/'controller.jsonl').exists() else []
    def first(event,name=None,arm=None):
        return next((e['timestamp'] for e in events if e['event']==event and
                     (name is None or e.get('name')==name) and (arm is None or e.get('arm')==arm)),None)
    information=next((e for e in events if e['event']=='information_ready'),None)
    observations=[e for e in events if e['event']=='action_start' and e.get('name','').startswith('observe')]
    arrivals=[e for e in events if e['event']=='action_end' and e.get('name','').startswith('observe')]
    config_hash=hashlib.sha256(json.dumps(config,sort_keys=True).encode()).hexdigest()
    summary=dict(result=result,config_id='v1_'+config_hash[:12],policy_id='v1_'+args.policy,
        instance_id=f'v1_seed_{args.seed}_{args.load_condition}',
        information_source=information['camera'] if information else None,
        observation_starts=observations,viewpoint_arrivals=arrivals,
        observation_cancellations=[e for e in events if e['event']=='action_cancel' and e.get('name','').startswith('observe')],
        t_install_command=first('installation_target'),
        t_insertion_ready=first('action_end','install_preapproach','right'),
        t_left_resume=next((e['timestamp'] for e in events if e['event']=='action_start' and e.get('arm')=='left' and e.get('name','').startswith('drawer')),None),
        t_right_exit=first('action_end','install_retreat','right'),
        observer_actions_started_after_information=[e for e in observations if information and e['timestamp']>=information['timestamp']],
        planning_failures=[e for e in events if e['event']=='planning_failure'],
        instrument_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        note='Passive snapshots do not advance physics. action_end timestamp marks next boundary; robot state there is sampled just before that last step.')
    save_json(path/'audit_summary.json',summary)
    for name,data in source_bytes.items():
        target=path/'instrumentation_source'/name
        target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
    save_json(path/'instrumentation_provenance.json',dict(source_sha256={name:hashlib.sha256(data).hexdigest()
              for name,data in source_bytes.items()},command=__import__('sys').argv))
    return result
