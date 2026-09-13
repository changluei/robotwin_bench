"""Post-hoc evidence tables for the finite development runs (does not run simulation)."""
import csv
import hashlib
import json
from collections import Counter
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from PIL import Image,ImageDraw,ImageFont

ROOT=Path(__file__).resolve().parents[2]
OUTPUT=ROOT/'experiments/held_module_key'
RUNS=ROOT/'result/held_module_key'


def jsonl(path):
    return [json.loads(x) for x in path.read_text().splitlines()] if path.exists() else []


def analyze(path):
    result=json.loads((path/'result.json').read_text())
    provenance=json.loads((path/'provenance.json').read_text())
    source=provenance['source_sha256']
    result.update(scene_source_id=source['envs/held_module_key/scene.py'][:12],
                  policy_source_id=source['envs/held_module_key/controller.py'][:12],
                  perception_source_id=source['envs/held_module_key/perception.py'][:12],
                  runtime_source_id=source['envs/held_module_key/runtime.py'][:12],
                  diagnostic_cycle='--cycle' in provenance['command'])
    result['canonical_instance_id']=f"{result['config_id']}/{result['scene_source_id']}/dev{result['seed']}"
    result['canonical_policy_id']=f"{result['policy_id']}/{result['policy_source_id']}"
    events=jsonl(path/'controller.jsonl');poses=jsonl(path/'evaluator_gt.jsonl')
    detections=jsonl(path/'perception.jsonl');contacts=jsonl(path/'contacts_gt.jsonl')
    valid=next((d for d in detections if d['valid']),None)
    if valid and poses:
        gt=min(poses,key=lambda r:abs(r['timestamp']-valid['timestamp']))
        measured=np.array(valid['T_world_key']);actual=np.array(gt['T_world_key'])
        result['first_visual_translation_error_gt']=float(np.linalg.norm(measured[:3,3]-actual[:3,3]))
        result['first_visual_rotation_error_degrees_gt']=float(np.rad2deg(Rotation.from_matrix(measured[:3,:3]@actual[:3,:3].T).magnitude()))
    result['events']=[e for e in events if e['event'] in ['observation_start','information_ready','action_cancel','action_end',
                                                        'installation_ready_physical','planning_failure','recovery_start','recovery_complete']]
    if result['mode']=='install' and result['success']:
        def tracking(action):
            t=next(e['timestamp'] for e in events if e['event']=='action_start' and e.get('name')==action)
            return next(e for e in events if e['event']=='robot_tracking' and e['arm']=='right' and e['timestamp']==t)
        pre=tracking('install_preapproach');insert=tracking('install_insert')
        a=np.array(insert['actual_fk']);b=np.array(pre['goal'])
        result['insertion_start_robot_audit']=dict(timestamp=insert['timestamp'],
            preposition_error_xyz=(a[:3,3]-b[:3,3]).tolist(),
            preorientation_error_degrees=float(np.rad2deg(Rotation.from_matrix(a[:3,:3]@b[:3,:3].T).magnitude())),
            note='Actual FK at insertion start; strict runtime 2 mm 3D stopped-readiness flag is kept separately, including null.')
    if poses:
        held=[r for r in poses if r['held_command'] and r['phase'] not in ['placing','regrasp']]
        release=next((e['timestamp'] for e in events if e['event']=='action_end' and e.get('name')=='install_release'),float('inf'))
        held=[r for r in held if r['timestamp']<release]
        result['jaw_range_gt']=np.ptp(np.array([r['jaw_qpos'] for r in held]),axis=0).tolist() if held else None
        result['max_held_linear_solver_velocity_gt']=max((float(np.linalg.norm(r['module_linear_velocity'])) for r in held),default=None)
        result['max_held_angular_solver_velocity_gt']=max((float(np.linalg.norm(r['module_angular_velocity'])) for r in held),default=None)
        result['final_key_error_gt']=dict(position=poses[-1]['key_position_error'],angle_degrees=poses[-1]['key_angle_error_degrees'])
        # No grip-state command may stand in for evidence of a physical regrasp.
        end_contacts=[c for c in contacts if c['timestamp']>=poses[-1]['timestamp']-.3]
        result['final_grasp_contact_gt']=bool(end_contacts) and all(all(any('key_module' in x['bodies'] and finger in x['bodies'] for x in c['contacts']) for finger in ['fr_link7','fr_link8']) for c in end_contacts)
        result['physical_recovery_verified']=bool(result.get('t_recovered') is not None and
                                                  (result['success'] if result['mode']=='install' else result['final_grasp_contact_gt']))
        if result['mode']=='geometry' and not result['physical_recovery_verified']:
            result['t_recovered_command']=result.get('t_recovered_command',result.get('t_recovered'))
            result['t_recovered']=None
    return result


def main():
    results=[analyze(p.parent) for p in sorted(RUNS.glob('*/result.json'))]
    (OUTPUT/'attempts.json').write_text(json.dumps(results,indent=2,ensure_ascii=False)+'\n')
    columns=['run_id','canonical_instance_id','canonical_policy_id','mode','diagnostic_cycle','initialized','attempt_completed','observation_valid','success',
             't_information','information_camera','t_install_ready_physical','t_install_ready','t_right_stable','t_left_restored','t_right_exit','t_recovered','failure_reason']
    with (OUTPUT/'attempts.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=columns,extrasaction='ignore');writer.writeheader();writer.writerows(results)
    stats={}
    for mode in ['geometry','install','planning']:
        rows=[r for r in results if r['mode']==mode]
        if not rows:continue
        stats[mode]=dict(attempts=len(rows),initialization_failures=sum(not r['initialized'] for r in rows),
                         initialized=sum(r['initialized'] for r in rows),protocol_completed=sum(r['attempt_completed'] for r in rows),
                         valid_observation=sum(r['observation_valid'] for r in rows),successful_endpoint=sum(r['success'] for r in rows))
    (OUTPUT/'statistics.json').write_text(json.dumps(stats,indent=2)+'\n')
    frozen=json.loads((OUTPUT/'PRESERVED_CONTROLS.json').read_text())
    changed=[name for name,sha in frozen.items() if not (ROOT/name).exists() or hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=sha]
    (OUTPUT/'preservation_check.json').write_text(json.dumps(dict(files_checked=len(frozen),changed=changed),indent=2)+'\n')
    print(json.dumps(stats,indent=2));print('preserved control changes',changed)


if __name__=='__main__':main()
