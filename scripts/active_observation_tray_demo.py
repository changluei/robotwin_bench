"""Phase-gated tray demo. Oracle geometry results are never baseline results."""
import argparse
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.environ.setdefault('MPLCONFIGDIR', '/tmp/robotwin-tray-mpl')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    probe = sub.add_parser('probe', help='Verify a real SAPIEN RGB-D capture')
    probe.add_argument('--output', type=Path, required=True)
    oracle = sub.add_parser('oracle', help='Phase 2 only: GT-driven physical geometry debug')
    oracle.add_argument('--seed', type=int, default=0)
    oracle.add_argument('--output', type=Path, required=True)
    oracle.add_argument('--physics-only', action='store_true',
                        help='Explicit debug mode without renderer; NOT a visual experiment')
    args = parser.parse_args()
    if args.command == 'probe':
        args.output.mkdir(parents=True, exist_ok=False)
        import sapien
        result = dict(render_available=False)
        try:
            scene = sapien.Scene([sapien.physx.PhysxCpuSystem(), sapien.render.RenderSystem()])
            camera = scene.add_camera('probe', 64, 64, 1., .015, 5)
            scene.update_render()
            camera.take_picture()
            result.update(render_available=True, rgb_shape=list(camera.get_picture('Color').shape),
                          depth_shape=list(camera.get_picture('Position')[...,2].shape))
        except Exception as exc:
            result['error'] = f'{type(exc).__name__}: {exc}'
        (args.output/'environment_probe.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result))
        return 0 if result['render_available'] else 1
    from envs.active_observation_tray.configs import Config
    from envs.active_observation_tray.debug_oracle import run
    result = run(Config(), args.seed, args.output, args.physics_only)
    return 0 if result['success'] else 1


if __name__ == '__main__':
    sys.exit(main())
