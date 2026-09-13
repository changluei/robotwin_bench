"""Offline physical/visual evidence audit, never imported by a policy."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from envs.occluded_socket.geometry import T_EE_MODULE,module_to_ee
from envs.occluded_socket.observation import CameraFrame
from envs.occluded_socket.perception import Perception


def read(path):return json.loads(path.read_text())
def lines(path):return [json.loads(x) for x in path.read_text().splitlines()] if path.exists() else []


def audit(root):
    rows=[]
    for file in sorted(root.rglob('result.json')):
        p=file.parent;r=read(file);record=dict(path=str(p),success=r['success'],mode=r['mode'])
        events=lines(p/'controller.jsonl');gt=lines(p/'evaluator_gt.jsonl')
        active={};errors=[]
        for e in events:
            if e['event']=='action_start':
                if e['arm'] in active:errors.append('overlapping actions')
                active[e['arm']]=e['name']
            if e['event'] in ['action_end','action_cancel']:
                if active.get(e['arm'])!=e['name']:errors.append('unmatched end/cancel')
                active.pop(e['arm'],None)
        record.update(action_order_errors=errors,unfinished_actions=active,
                      failure_completion_is_null=r['success'] or r['t_all_complete'] is None)
        record['last_physical_sample']=next((g for g in reversed(gt) if 'module_error_xyz' in g),None)
        initial=read(p/'initial_state_gt.json') if (p/'initial_state_gt.json').exists() else None
        config=read(p/'config.json')
        if (p/'initialization_gt.json').exists():
            init=read(p/'initialization_gt.json')
            T=np.array(init['measured_T_ee_module_evaluation_only'])
            record['initial_grasp_rotation_error_deg']=float(np.rad2deg(Rotation.from_matrix(T_EE_MODULE[:3,:3].T@T[:3,:3]).magnitude()))
            record['initial_grasp_translation_error_metres']=float(np.linalg.norm(T[:3,3]-T_EE_MODULE[:3,3]))
        info=next((e for e in events if e['event']=='information_ready'),None)
        target=next((e for e in events if e['event']=='installation_target'),None)
        if info and initial:
            estimate=np.array(info['T_world_socket'])
            record['translation_error_metres']=float(np.linalg.norm(estimate[:3,3]-np.array(initial['socket_pose'])[:3,3]))
            if target:
                expected=estimate.copy();expected[2,3]+=config['module_half_size'][2]+.002
                record['installation_transform_error']=float(np.max(abs(module_to_ee(expected)-np.array(target['T_world_ee_goal']))))
            image=p/'audit_frames/first_information'/f"{info['camera']}.npz"
            if image.exists():
                obs=np.load(image);perception=Perception(config['marker_points'],config['perception'])
                frame=CameraFrame('masked',0,np.zeros_like(obs['rgb']),obs['depth'],obs['intrinsic'],obs['T_world_camera'])
                record['masked_evidence_rejected']=all(not perception.detect(frame).valid for _ in range(3))
        contacts=lines(p/'contacts_gt.jsonl');other={};rack=[]
        for row in contacts:
            for contact in row['contacts']:
                names=set(contact['bodies'])
                if names=={'module','temporary_rack'}:rack.append(row['timestamp'])
                intended=(('module' in names and bool(names&{'fr_link7','fr_link8','hidden_socket','temporary_rack'})) or
                          ('drawer_slide' in names and bool(names&{'fl_link7','fl_link8','fr_link7','fr_link8'})))
                if not intended and any(n.startswith(('fl_link','fr_link')) for n in names):
                    key=' / '.join(sorted(names));other[key]=max(other.get(key,0.),contact['impulse'])
        record['other_robot_contacts_max_impulse']=other
        record['module_rack_contact_range']=[min(rack),max(rack)] if rack else None
        if initial and r['policy']=='H' and r['load_condition']=='HIGH' and info:
            values=[g['drawer_position'] for g in gt if 'drawer_position' in g and g['timestamp']<=info['timestamp']]
            record['drawer_motion_before_information']=float(max(abs(np.array(values)-initial['drawer_qpos'][0]))) if values else None
        rows.append(record)
    out=root/'physics_evidence_audit.json'
    out.write_text(json.dumps(dict(runs=rows,note='Offline ground truth only. Contacts sampled at 50 Hz above 0.0001 N s, not exhaustive collision proof.'),indent=2))
    print(json.dumps(dict(runs=len(rows),action_order_failures=sum(bool(r['action_order_errors']) for r in rows),output=str(out))))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path)
    audit(p.parse_args().root.resolve())
