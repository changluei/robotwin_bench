"""Small development-only audits and geometry calibration; preserves all attempts."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode',choices=['audit','audit-suite','route','route-suite','oracle'])
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seed',type=int,default=0)
    p.add_argument('--policy',choices=['H','S'],default='H')
    p.add_argument('--load-condition',choices=['LOW','HIGH'],default='LOW')
    p.add_argument('--gpu',type=int,default=0)
    p.add_argument('--record',action='store_true')
    p.add_argument('--route',default='direct_live')
    p.add_argument('--routes',default='helper_original,helper_near,helper_front,helper_high,self_small_x,self_small_yaw,self_small_back,self_large_x,self_original,self_large_side,direct_live,direct_initial_only,putdown,putdown_cycle_validation')
    p.add_argument('--scene',choices=['v1','v2','v2_clearance','v2_baffle'],default='v1')
    args=p.parse_args();os.chdir(ROOT)
    args.output=args.output.resolve()
    if args.mode=='route-suite':
        args.output.mkdir(parents=True,exist_ok=False)
        for route in args.routes.split(','):
            run=args.output/f's{args.seed}_{args.load_condition}_{route}'
            cmd=[sys.executable,str(Path(__file__).resolve()),'route','--seed',str(args.seed),
                 '--load-condition',args.load_condition,'--route',route,'--gpu',str(args.gpu),
                 '--scene',args.scene,'--output',str(run)]
            if args.record:cmd+=['--record']
            with (args.output/f'{run.name}.console.log').open('w') as log:
                proc=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT)
            print(json.dumps(dict(run=str(run),process_exit=proc.returncode)),flush=True)
        return
    if args.mode=='audit-suite':
        args.output.mkdir(parents=True,exist_ok=False)
        # Existing development 0..4 plus already analyzed representative 104.
        # These are diagnostic reruns, never a new independent test cohort.
        for seed in [0,1,2,3,4,104]:
            for condition in (['LOW','HIGH'] if seed==0 else ['LOW']):
                for policy in ['H','S']:
                    run=args.output/f's{seed}_{condition}_{policy}'
                    cmd=[sys.executable,str(Path(__file__).resolve()),'audit','--seed',str(seed),
                         '--load-condition',condition,'--policy',policy,'--gpu',str(args.gpu),'--output',str(run)]
                    if args.record:cmd+=['--record']
                    with (args.output/f'{run.name}.console.log').open('w') as log:
                        proc=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT)
                    print(json.dumps(dict(run=str(run),process_exit=proc.returncode)),flush=True)
        return
    import yaml
    from envs.occluded_socket_calibration.audit import instrumented_episode
    config_name={'v1':'occluded_socket_drawer.yml','v2':'occluded_socket_drawer_v2_probe.yml',
                 'v2_clearance':'occluded_socket_drawer_v2_clearance.yml',
                 'v2_baffle':'occluded_socket_drawer_v2_baffle.yml'}[args.scene]
    config=yaml.safe_load((ROOT/'env_cfg/task_config'/config_name).read_text(encoding='utf-8'))
    if args.scene.startswith('v2'):
        if args.load_condition!='LOW':raise ValueError('v2 HIGH is gated on a demonstrated LOW helper benefit')
        from envs.occluded_socket_calibration.scene import OcclusionChannelScene,ChannelKinematics
        from envs.occluded_socket import runtime
        runtime.occluded_socket_drawer=OcclusionChannelScene
        runtime.RobotKinematics=ChannelKinematics
    route=None
    if args.mode=='route':
        from envs.occluded_socket_calibration.controller import ROUTES,RouteController
        from envs.occluded_socket import runtime
        route=ROUTES[args.route]
        args.policy='H' if route['kind'].startswith('helper') else 'S'
        runtime.Controller=lambda *a,**kw:RouteController(*a,**kw,route=route)
    args.mode='oracle' if args.mode=='oracle' else 'episode'
    args.headless=True;args.right_only=args.mode=='oracle';args.variant='baseline'
    result=instrumented_episode(args,config)
    if route:
        from envs.occluded_socket.recording import save_json
        import hashlib
        config_id=args.scene+'_'+hashlib.sha256(json.dumps(config,sort_keys=True).encode()).hexdigest()[:12]
        result.update(policy_id=args.route,config_id=config_id,
                      instance_id=f'{config_id}/seed_{args.seed}/{args.load_condition}',route=route,
                      diagnostic_only=bool(route.get('force_cycle',False) or route.get('initial_only',False)))
        save_json(args.output/'result.json',result)
        save_json(args.output/'route.json',dict(name=args.route,**route))


if __name__=='__main__':
    main()
