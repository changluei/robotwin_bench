"""All-attempt statistics for version-isolated development calibration."""
import argparse
import csv
import json
import hashlib
from pathlib import Path
import numpy as np


def summarize(root):
    rows=[]
    for file in sorted(root.rglob('result.json')):
        run=file.parent;r=json.loads(file.read_text())
        audit=json.loads((run/'audit_summary.json').read_text()) if (run/'audit_summary.json').exists() else {}
        config=json.loads((run/'config.json').read_text()) if (run/'config.json').exists() else {}
        canonical_config=('v2_' if 'front_occlusion_wall' in config else 'v1_')+hashlib.sha256(json.dumps(config,sort_keys=True).encode()).hexdigest()[:12]
        row=dict(path=str(run),attempt_id=str(run.relative_to(root)),seed=r['seed'],
            config_id=canonical_config,declared_config_id=r.get('config_id',audit.get('config_id','unknown')),
            instance_id=f"{canonical_config}/seed_{r['seed']}/{r['load_condition']}",
            policy_id='ORACLE_ONLY' if r.get('mode')=='oracle' else r.get('policy_id',audit.get('policy_id',r['policy'])),
            mode=r.get('mode'),
            load_condition=r['load_condition'],initialized=(run/'initial_state_gt.json').exists(),
            task_success=r['success'],failure_reason=r.get('failure_reason'),
            initial_state_sha256=r.get('initial_state_sha256'),information_source=audit.get('information_source'),
            t_information_ready=r.get('t_information_ready'),t_insertion_ready=audit.get('t_insertion_ready'),
            t_left_resume=audit.get('t_left_resume'),t_right_complete=r.get('t_right_complete'),
            t_right_exit=audit.get('t_right_exit'),t_all_complete=r.get('t_all_complete'),
            diagnostic_only=r.get('diagnostic_only',False) or r.get('mode')=='oracle')
        info=run/'audit_frames/first_information/state.json'
        row['motion_before_information']=json.loads(info.read_text())['motion_before_event'] if info.exists() else None
        events=[json.loads(x) for x in (run/'controller.jsonl').read_text().splitlines()] if (run/'controller.jsonl').exists() else []
        for name in ['putdown_not_needed','module_on_rack_command_complete','held_installation_restored']:
            row['t_'+name]=next((e['timestamp'] for e in events if e['event']==name),None)
        row['observation_after_information']=audit.get('observer_actions_started_after_information')
        rows.append(row)
    out=root/'summary';out.mkdir(exist_ok=True)
    with (out/'attempts.csv').open('w',newline='',encoding='utf-8') as f:
        fields=[k for k in rows[0] if k not in ['motion_before_information','observation_after_information']] if rows else []
        writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(rows)
    groups={}
    for key in sorted({(r['config_id'],r['policy_id'],r['load_condition']) for r in rows}):
        group=[r for r in rows if (r['config_id'],r['policy_id'],r['load_condition'])==key]
        initialized=sum(r['initialized'] for r in group);ok=sum(r['task_success'] for r in group)
        groups['/'.join(key)]=dict(attempts=len(group),initialized=initialized,initialization_failures=len(group)-initialized,
             task_successes=ok,task_success_rate_given_initialized=ok/initialized if initialized else None,
             all_attempt_task_completion_fraction=ok/len(group))
    (out/'summary.json').write_text(json.dumps(dict(groups=groups,runs=rows,
        note='Development diagnostics, all reruns retained; no independent test claim. Insertion-ready is completion of the shared preapproach, not merely first detection.'),indent=2))
    lines=['# Actual calibration attempts','', '| Instance | Config | Policy | Initialized | Success | Info source | Info | Insertion ready | Right stable | Right exit | Failure |',
        '|---|---|---|---|---|---|---:|---:|---:|---:|---|']
    for r in rows:
        lines.append('| '+' | '.join(str(r.get(k)) for k in ['instance_id','config_id','policy_id','initialized','task_success','information_source','t_information_ready','t_insertion_ready','t_right_complete','t_right_exit','failure_reason'])+' |')
    (out/'summary.md').write_text('\n'.join(lines)+'\n')
    # Native scientific figure from real rendered frames, never generated images.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for seed in [0,104]:
        chosen=[]
        for policy in ['v1_H','v1_S']:
            r=next((r for r in rows if r['seed']==seed and r['policy_id']==policy and r['load_condition']=='LOW'),None)
            if r:chosen.append(r)
        if not chosen:continue
        samples=[(Path(chosen[0]['path'])/'audit_frames/initial','initial t=0')]
        samples.extend((Path(r['path'])/'audit_frames/first_information',f"{r['policy_id']} first valid t={r['t_information_ready']} from {r['information_source']}") for r in chosen)
        fig,axes=plt.subplots(len(samples),3,figsize=(13,3.5*len(samples)),squeeze=False)
        for i,(folder,label) in enumerate(samples):
            for j,camera in enumerate(['head','left','right']):
                image=folder/f'{camera}.png'
                if image.exists():axes[i,j].imshow(plt.imread(image))
                axes[i,j].set_title(f'{label}\n{camera}');axes[i,j].axis('off')
        fig.tight_layout();fig.savefig(out/f'seed{seed}_actual_cameras.png',dpi=150);plt.close(fig)
    if any(not r['policy_id'].startswith('v1_') for r in rows):
        fig,ax=plt.subplots(figsize=(12,max(4,len(rows)*.34)))
        for i,r in enumerate(rows):
            label=f"s{r['seed']} {r['policy_id']}"+(' [failed]' if not r['task_success'] else '')
            for k,color in [('t_information_ready','#416eaf'),('t_insertion_ready','#df9b40'),('t_right_complete','#428d63')]:
                if r[k] is not None:ax.scatter(r[k],i,c=color,marker='o',label=k if i==0 else None)
            ax.text(-.2,i,label,ha='right',va='center',fontsize=8)
        ax.set(xlabel='Actual physics time (s)',yticks=[]);ax.invert_yaxis();ax.legend()
        fig.subplots_adjust(left=.34);fig.savefig(out/'time_decomposition.png',dpi=150);plt.close(fig)
    print(json.dumps(groups),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path)
    summarize(p.parse_args().root.resolve())
