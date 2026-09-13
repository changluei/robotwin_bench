"""Offline evidence checks. Reads GT here only; never used during control."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from envs.occluded_socket.observation import CameraFrame
from envs.occluded_socket.perception import Perception
from envs.occluded_socket.geometry import module_to_ee


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def jsonl(path):
    return [json.loads(x) for x in path.read_text(encoding='utf-8').splitlines()] if path.exists() else []


def audit(root):
    results=[]
    instances={}
    manifest=ROOT/'experiments/occluded_socket_drawer/FROZEN.json'
    frozen=read_json(manifest)['source_sha256'] if manifest.exists() else None
    for file in sorted(root.rglob('result.json')):
        result=read_json(file)
        if result.get('mode')!='episode':
            continue
        run=file.parent
        cohort=run.parent.name
        events=jsonl(run/'controller.jsonl')
        joints=jsonl(run/'joints.jsonl')
        gt=jsonl(run/'evaluator_gt.jsonl')
        initial=read_json(run/'initial_state_gt.json') if (run/'initial_state_gt.json').exists() else None
        info=next((e for e in events if e['event']=='information_ready'),None)
        record=dict(run_id=result['run_id'],run_path=str(run),attempt_cohort=cohort,
                    success=result['success'],seed=result['seed'],t_information_ready=result['t_information_ready'],
                    policy=result['policy'],load_condition=result['load_condition'])
        provenance=read_json(run/'provenance.json') if (run/'provenance.json').exists() else None
        record['frozen_source_match']=provenance['source_sha256']==frozen if frozen and provenance else None
        record['snapshot_hashes_match_provenance']=all((run/'source'/p).exists() and
            hashlib.sha256((run/'source'/p).read_bytes()).hexdigest()==sha for p,sha in provenance['source_sha256'].items()) if provenance else None
        if initial:
            key=(cohort,result['seed'],result['load_condition'],result['policy'])
            if key in instances:
                raise ValueError(f'Duplicate physical run: {key}')
            instances[key]=initial
        active={}
        ordering=[]
        for event in events:
            if event['event']=='action_start':
                if event['arm'] in active:
                    ordering.append(f'overlapping actions on {event["arm"]}')
                active[event['arm']]=event['name']
            if event['event'] in ['action_end','action_cancel']:
                if active.get(event['arm'])!=event['name']:
                    ordering.append(f'unmatched {event["event"]}: {event["name"]}')
                active.pop(event['arm'],None)
        record['action_order_errors']=ordering
        record['unfinished_actions']=active
        record['joint_timestamps_monotonic']=all(b['timestamp']>a['timestamp'] for a,b in zip(joints,joints[1:]))
        record['parallel_manipulation_samples']=sum(r['moving']['left'] and r['moving']['right'] and
                       r['left_state'] in ['drawer_hover','drawer_approach','drawer_contact','drawer_push','drawer_retreat']
                       and r['right_state'] in ['install_preapproach','install_insert','install_release','install_retreat'] for r in joints)
        record['sampling_period_seconds']=float(np.median(np.diff([r['timestamp'] for r in joints]))) if len(joints)>1 else None
        record['completion_null_on_failure']=result['success'] or result['t_all_complete'] is None
        if (run/'camera_calibration.json').exists():
            record['camera_matrix_max_error']=max(c['matrix_max_error'] for c in read_json(run/'camera_calibration.json').values())
        if info and initial:
            estimate=np.array(info['T_world_socket'])[:3,3]
            truth=np.array(initial['socket_pose'])[:3,3]
            record['information_source']=info['camera']
            record['estimate_xyz']=estimate.tolist();record['true_xyz']=truth.tolist()
            record['translation_error_metres']=float(np.linalg.norm(estimate-truth))
            target=next((e for e in events if e['event']=='installation_target'),None)
            if target:
                record['installation_ee_xyz']=np.array(target['T_world_ee_goal'])[:3,3].tolist()
                expected=np.array(info['T_world_socket']).copy()
                expected[2,3]+=read_json(run/'config.json')['module_half_size'][2]+.002
                record['installation_target_transform_max_error']=float(np.max(abs(
                    module_to_ee(expected)-np.array(target['T_world_ee_goal']))))
            if result['policy']=='H' and result['load_condition']=='HIGH':
                values=[r['drawer_position'] for r in gt if 'drawer_position' in r and r['timestamp']<=info['timestamp']]
                record['drawer_motion_before_information']=max(abs(np.array(values)-initial['drawer_qpos'][0])) if values else None
        record['last_sampled_physical_state']=next((g for g in reversed(gt) if 'module_error_xyz' in g),None)
        record['masked_image_rejected']=None
        observations=sorted(run.glob('valid_*_observation.npz'))
        if observations:
            obs=np.load(observations[0]);config=read_json(run/'config.json')
            perception=Perception(config['marker_points'],config['perception'])
            frame=CameraFrame('masked',0,np.zeros_like(obs['rgb']),obs['depth'],obs['intrinsic'],obs['T_world_camera'])
            detections=[perception.detect(frame) for _ in range(3)]
            record['masked_image_rejected']=all(not d.valid and d.T_world_socket is None for d in detections)
        contacts=jsonl(run/'contacts_gt.jsonl')
        unexpected={}
        for row in contacts:
            for contact in row['contacts']:
                names=set(contact['bodies'])
                intended=(('module' in names and (names&{'fr_link7','fr_link8','hidden_socket'})) or
                          ('drawer_slide' in names and (names&{'fl_link7','fl_link8','fr_link7','fr_link8'})))
                if not intended and any(n.startswith(('fl_link','fr_link')) for n in names):
                    key=' / '.join(sorted(names))
                    unexpected[key]=max(unexpected.get(key,0),contact['impulse'])
        record['other_robot_contact_max_impulses']=unexpected
        results.append(record)
    pair_checks=[]
    right_checks=[]
    for cohort,seed in sorted({key[:2] for key in instances}):
        for load in ['LOW','HIGH']:
            a=instances.get((cohort,seed,load,'H'));b=instances.get((cohort,seed,load,'S'))
            if a and b:
                pair_checks.append(dict(seed=seed,condition=load,attempt_cohort=cohort,exact_match=a==b))
        for policy in ['H','S']:
            a=instances.get((cohort,seed,'LOW',policy));b=instances.get((cohort,seed,'HIGH',policy))
            if a and b:
                ids=[i for i,n in enumerate(a['joint_names']) if n.startswith('fr_joint')]
                error=max(float(np.max(abs(np.array(a[k])-np.array(b[k])))) for k in ['socket_pose','module_pose','module_velocity'])
                error=max(error,float(np.max(abs(np.array(a['qpos'])[ids]-np.array(b['qpos'])[ids]))))
                right_checks.append(dict(seed=seed,policy=policy,attempt_cohort=cohort,right_initial_max_difference=error))
    estimates=np.array([r['estimate_xyz'] for r in results if 'estimate_xyz' in r])
    targets=np.array([r['installation_ee_xyz'] for r in results if 'installation_ee_xyz' in r])
    report=dict(runs=results,paired_initial_states=pair_checks,low_high_right_states=right_checks,
                estimated_target_xy_span=np.ptp(estimates[:,:2],axis=0).tolist() if len(estimates) else None,
                installation_ee_xy_span=np.ptp(targets[:,:2],axis=0).tolist() if len(targets) else None,
                note='Offline evaluator data; these checks are never policy inputs. Motion impulses are N s, not forces.')
    out=root/'evidence_audit.json';out.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(dict(runs=len(results),paired_exact=sum(p['exact_match'] for p in pair_checks),
                pairs=len(pair_checks),max_camera_error=max([r.get('camera_matrix_max_error',0) for r in results],default=0),
                max_translation_error=max([r.get('translation_error_metres',0) for r in results],default=0),
                unexpected_contacts={r['run_id']:r['other_robot_contact_max_impulses'] for r in results if r['other_robot_contact_max_impulses']},
                output=str(out))),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('root',type=Path)
    audit(parser.parse_args().root.resolve())
