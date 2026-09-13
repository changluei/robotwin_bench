"""One geometry path per invocation; no HIGH, training, selector, or batch mode."""
import argparse
from pathlib import Path
import sys
import os
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));os.chdir(ROOT)

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route',choices=['hold_translate','helper','head','passive_left','putdown'],required=True)
    parser.add_argument('--pick',type=int,default=0)
    parser.add_argument('--candidates-from',type=Path,help='Reuse a saved public robot-only candidate list; trajectories are checked again at execution.')
    parser.add_argument('--joint-search',action='store_true',help='Bounded alternative joint poses for head/fixed-left presentation.')
    parser.add_argument('--seed',type=int,default=400)
    parser.add_argument('--gpu',type=int,default=0)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--install',action='store_true')
    parser.add_argument('--cycle',action='store_true',help='Diagnostic putdown cycle; separately labelled, never mandatory S.')
    parser.add_argument('--plan',action='store_true',help='Robot-only bounded route feasibility, no physical route execution.')
    args=parser.parse_args()
    import yaml
    from envs.held_module_key.runtime import run
    config=yaml.safe_load((ROOT/'env_cfg/task_config/held_module_key.yml').read_text(encoding='utf-8'))
    run(args,config)
