"""Summarize all outcomes; failed completion times stay null. No seed filtering."""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import numpy as np


def summarize(root, output, compose=False):
    output.mkdir(parents=True,exist_ok=True)
    paths=sorted(root.rglob('result.json'))
    records=[(p,json.loads(p.read_text(encoding='utf-8'))) for p in paths]
    records=[(p,r) for p,r in records if r.get('mode')=='episode']
    rows=[dict(r,path=str(p.parent),attempt_cohort=p.parent.parent.name) for p,r in records]
    fields=['run_id','seed','policy','load_condition','success','timeout','failure_reason',
            't_information_ready','t_right_complete','t_left_complete','t_all_complete',
            'both_arms_moving_seconds','path','attempt_cohort']
    with (output/'results.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(rows)
    summary={}
    budgets=[5,8,10,12,15,20,25,30,35]
    for load in ['LOW','HIGH']:
        for policy in ['H','S']:
            group=[r for r in rows if r['load_condition']==load and r['policy']==policy]
            failures={}
            for r in group:
                if not r['success']:
                    reason=r.get('failure_reason','unknown');failures[reason]=failures.get(reason,0)+1
            ok=[r for r in group if r['success']]
            summary[f'{load}_{policy}']=dict(n=len(group),successes=len(ok),failures=failures,
                success_at_budget={str(b):sum(r['success'] and r['t_all_complete'] is not None and r['t_all_complete']<=b for r in group)/len(group) if group else None for b in budgets},
                successful_mean_times={k:float(np.mean([r[k] for r in ok if r.get(k) is not None])) if any(r.get(k) is not None for r in ok) else None for k in fields[7:11]})
    pairs=[]
    cohorts={}
    for cohort in sorted({r['attempt_cohort'] for r in rows}):
        cohort_rows=[r for r in rows if r['attempt_cohort']==cohort]
        cohorts[cohort]=dict(attempts=len(cohort_rows),successes=sum(r['success'] for r in cohort_rows),
            initialized=sum(r.get('initial_state_sha256') is not None for r in cohort_rows))
        for seed in sorted({r['seed'] for r in rows}):
            for load in ['LOW','HIGH']:
                members=[r for r in rows if r['seed']==seed and r['load_condition']==load and r['attempt_cohort']==cohort]
                g={r['policy']:r for r in members}
                if set(g)!={'H','S'}:
                    continue
                if len(members)!=2:
                    raise ValueError(f'Duplicate runs in cohort {cohort}, seed {seed}, {load}; use distinct attempt directories')
                matched=(g['H'].get('initial_state_sha256') is not None and
                         g['H'].get('initial_state_sha256')==g['S'].get('initial_state_sha256'))
                both=g['H']['success'] and g['S']['success']
                available=all(g[p].get('initial_state_sha256') is not None for p in ['H','S'])
                pairs.append(dict(seed=seed,load_condition=load,attempt_cohort=cohort,both_success=both,
                    initial_state_available=available,initial_state_exact_match=matched,
                    H_minus_S=(g['H']['t_all_complete']-g['S']['t_all_complete']) if both else None))
    paired_statistics={}
    for load in ['LOW','HIGH']:
        differences=[p['H_minus_S'] for p in pairs if p['load_condition']==load and p['both_success']]
        paired_statistics[load]=dict(n=len(differences),
            mean=float(np.mean(differences)) if differences else None,
            median=float(np.median(differences)) if differences else None,
            minimum=min(differences,default=None),maximum=max(differences,default=None),
            H_faster=sum(d < -1e-9 for d in differences),ties=sum(abs(d)<=1e-9 for d in differences),
            S_faster=sum(d > 1e-9 for d in differences))
    reversals=[]
    for p in pairs:
        if p['load_condition']!='LOW' or not p['both_success']:
            continue
        high=next((q for q in pairs if q['attempt_cohort']==p['attempt_cohort'] and
                   q['seed']==p['seed'] and q['load_condition']=='HIGH' and q['both_success']),None)
        if high and p['H_minus_S'] < -1e-9 and high['H_minus_S'] > 1e-9:
            reversals.append(dict(seed=p['seed'],attempt_cohort=p['attempt_cohort'],
                LOW_H_minus_S=p['H_minus_S'],HIGH_H_minus_S=high['H_minus_S']))
    result=dict(groups=summary,cohorts=cohorts,pairs=pairs,paired_time_statistics=paired_statistics,
                raw_sign_reversals=reversals,
                paired_time_condition='H and S both successful; all outcomes remain in success denominators',
                time_convention='physics sum(dt), includes safe retreats; planning wall-clock excluded')
    (output/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(15,4),constrained_layout=True)
    colors={'LOW_H':'#4185bb','LOW_S':'#e69538','HIGH_H':'#244765','HIGH_S':'#a25b17'}
    for name,g in summary.items():
        if g['n']:
            axes[0].plot(budgets,list(g['success_at_budget'].values()),'o-',label=f'{name}: {g["successes"]}/{g["n"]}',color=colors[name])
    axes[0].set(xlabel='Physics budget (s)',ylabel='Success rate',ylim=(-.03,1.03));axes[0].legend(fontsize=8)
    for i,load in enumerate(['LOW','HIGH']):
        valid=[p for p in pairs if p['load_condition']==load and p['both_success']]
        axes[1].scatter([p['seed'] for p in valid],[p['H_minus_S'] for p in valid],label=f'{load} (both succeed)',marker=['o','x'][i])
    axes[1].axhline(0,color='gray',lw=1);axes[1].set(xlabel='Seed',ylabel='H - S completion time (s)');axes[1].legend(fontsize=8)
    keys=['t_information_ready','t_right_complete','t_left_complete','t_all_complete']
    for offset,(name,g) in enumerate(summary.items()):
        values=[g['successful_mean_times'].get(k) for k in keys]
        axes[2].plot(range(4),[np.nan if v is None else v for v in values],'o-',label=name,color=colors[name])
    axes[2].set_xticks(range(4),['information','right','left','all'],rotation=20)
    axes[2].set_ylabel('Mean time, successful runs only (s)');axes[2].legend(fontsize=8)
    fig.savefig(output/'results.png',dpi=160);fig.savefig(output/'results.pdf');plt.close(fig)
    lines=['# Actual execution summary','',f'Visual runs: {len(rows)}. No failures removed.','',
           '| Group | Success | Failure reasons | Mean all-complete (successful only) |','|---|---:|---|---:|']
    for name,g in summary.items():
        lines.append(f'| {name} | {g["successes"]}/{g["n"]} | {g["failures"]} | {g["successful_mean_times"].get("t_all_complete")} |')
    lines+=['','Paired time comparisons condition on both strategies succeeding. Positive H−S means S is faster.',
            f'Initial states exactly match for {sum(p["initial_state_exact_match"] for p in pairs)}/{sum(p["initial_state_available"] for p in pairs)} initialized H/S pairs; {sum(not p["initial_state_available"] for p in pairs)} pairs lack initial states (startup failures).',
            f'Attempt cohorts: {cohorts}',f'Paired H minus S statistics: {paired_statistics}',
            f'Raw LOW-negative/HIGH-positive sign changes (inspect magnitude before interpretation): {reversals}','']
    (output/'summary.md').write_text('\n'.join(lines),encoding='utf-8')
    if compose:
        import imageio_ffmpeg
        import imageio.v2 as imageio
        seed=min(r['seed'] for r in rows)
        videos=[]
        for load,policy in [('LOW','H'),('LOW','S'),('HIGH','H'),('HIGH','S')]:
            chosen=next((Path(r['path'])/'execution.mp4' for r in rows if r['seed']==seed and r['load_condition']==load and r['policy']==policy),None)
            if chosen is None or not chosen.exists():
                raise RuntimeError('Four original videos required for composition')
            videos.append(chosen)
        physics_duration=max(r['terminal_physics_time'] for r in rows if r['seed']==seed)
        # A terminal render between camera ticks occupies the next video frame.
        # Keep that source frame too; truncating at physics_duration can cut it off.
        durations=[]
        for video in videos:
            reader=imageio.get_reader(video)
            try:
                fps=reader.get_meta_data()['fps']
                if abs(fps-10)>1e-6:
                    raise ValueError('Composition requires all source videos at 10 fps')
                durations.append(reader.count_frames()/fps)
            finally:
                reader.close()
        duration=max(durations)
        cmd=[imageio_ffmpeg.get_ffmpeg_exe(),'-y','-loglevel','error','-threads','2']
        for video in videos:
            cmd+=['-i',str(video)]
        filters=';'.join(f'[{i}:v]setpts=PTS-STARTPTS,tpad=stop_mode=clone:stop_duration={duration}[v{i}]' for i in range(4))
        filters+=';[v0][v1][v2][v3]xstack=inputs=4:layout=0_0|960_0|0_720|960_720[out]'
        cmd+=['-filter_complex_threads','1','-filter_complex',filters,'-map','[out]','-t',str(duration),
              '-r','10','-c:v','libx264','-preset','fast','-crf','21','-pix_fmt','yuv420p',str(output/'four_way.mp4')]
        subprocess.run(cmd,check=True)
        (output/'video_alignment.json').write_text(json.dumps(dict(seed=seed,inputs=list(map(str,videos)),
                  fps=10,speed=1,physics_duration=physics_duration,video_duration=duration,
                  alignment='checkpoint t=0; completed videos hold final rendered frame; terminal frames retained at original frame cadence',command=cmd),indent=2))
    print(json.dumps(result),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('root',type=Path);p.add_argument('--output',type=Path);p.add_argument('--compose',action='store_true')
    args=p.parse_args();summarize(args.root.resolve(),(args.output or args.root/'summary').resolve(),args.compose)
