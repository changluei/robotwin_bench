"""Reproducible audit / oracle / visual / paired evaluation entry point."""
import argparse
import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode',choices=['audit','oracle','drawer','episode','paired','batch'])
    p.add_argument('--policy',choices=['H','S'],default='H')
    p.add_argument('--load-condition',choices=['LOW','HIGH'],default='LOW')
    p.add_argument('--seed',type=int,default=0)
    p.add_argument('--gpu',type=int,default=0)
    p.add_argument('--headless',action=argparse.BooleanOptionalAction,default=True)
    p.add_argument('--record',action=argparse.BooleanOptionalAction,default=False)
    p.add_argument('--right-only',action='store_true')
    p.add_argument('--output',type=Path)
    p.add_argument('--config',type=Path,default=ROOT/'env_cfg/task_config/occluded_socket_drawer.yml')
    p.add_argument('--budget',type=float)
    p.add_argument('--seeds',default='100:120',help='inclusive start, exclusive end; default independent test seeds')
    p.add_argument('--variant',choices=['baseline','right_handoff','partial_helper'],default='baseline')
    args=p.parse_args()
    os.chdir(ROOT)
    os.environ.setdefault('PYTHONUTF8','1')
    if args.output is None:
        stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        args.output=ROOT/'result/occluded_socket_drawer'/f'{stamp}_{args.mode}_{args.policy}_{args.load_condition}_s{args.seed}'
    args.output=args.output.resolve()
    if args.mode=='audit':
        args.output.mkdir(parents=True,exist_ok=False)
        commands={'git':['git','status','--short'],'commit':['git','rev-parse','HEAD'],
                  'gpu':['nvidia-smi','--query-gpu=index,name,uuid,memory.used,memory.total,utilization.gpu','--format=csv']}
        for name,cmd in commands.items():
            (args.output/f'{name}.txt').write_text(subprocess.check_output(cmd,text=True),encoding='utf-8')
        from envs.occluded_socket.runtime import code_version
        (args.output/'code.json').write_text(json.dumps(code_version(),indent=2),encoding='utf-8')
        cmd=[sys.executable,str(ROOT/'scripts/preview_shelf_scene.py'),'--seed','0','--output',str(args.output/'smoke')]
        with (args.output/'smoke.log').open('w') as f:
            result=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,env={**os.environ,'CUDA_VISIBLE_DEVICES':str(args.gpu)})
        print('existing task smoke exit',result.returncode,'output',args.output)
        return
    if args.mode in ['paired','batch']:
        args.output.mkdir(parents=True,exist_ok=False)
        seeds=[args.seed] if args.mode=='paired' else list(range(*map(int,args.seeds.split(':'))))
        rows=[]
        for seed in seeds:
            for condition in ['LOW','HIGH']:
                for policy in ['H','S']:
                    out=args.output/f's{seed}_{condition}_{policy}'
                    cmd=[sys.executable,str(Path(__file__).resolve()),'episode','--policy',policy,
                         '--load-condition',condition,'--seed',str(seed),'--gpu',str(args.gpu),
                         '--config',str(args.config),'--output',str(out),'--variant',args.variant,
                         '--record' if args.record else '--no-record','--headless' if args.headless else '--no-headless']
                    if args.budget is not None:
                        cmd+=['--budget',str(args.budget)]
                    with (args.output/f'{out.name}.console.log').open('w') as f:
                        proc=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
                    if (out/'result.json').exists():
                        row=json.loads((out/'result.json').read_text())
                    else:
                        row=dict(run_id=out.name,seed=seed,policy=policy,load_condition=condition,success=False,
                                 mode='episode',variant=args.variant,timeout=False,t_information_ready=None,
                                 t_right_complete=None,t_left_complete=None,
                                 failure_reason=f'process_exit_{proc.returncode}',t_all_complete=None)
                        out.mkdir(parents=True,exist_ok=True)
                        (out/'result.json').write_text(json.dumps(row,indent=2),encoding='utf-8')
                    rows.append(row)
                    (args.output/'results.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
                    print(json.dumps(row),flush=True)
        return
    import yaml
    config=yaml.safe_load(args.config.read_text(encoding='utf-8'))
    if args.budget is not None:
        config['budget']=args.budget
    from envs.occluded_socket.runtime import run_episode
    run_episode(args,config)

if __name__=='__main__':
    main()
