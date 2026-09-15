"""Pure-RGB active-observation tray experiment."""
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
    probe = sub.add_parser('probe', help='Verify a real SAPIEN RGB capture')
    probe.add_argument('--output', type=Path, required=True)
    oracle = sub.add_parser('oracle', help='Phase 2 only: GT-driven physical geometry debug')
    oracle.add_argument('--seed', type=int, default=0)
    oracle.add_argument('--output', type=Path, required=True)
    oracle.add_argument('--physics-only', action='store_true',
                        help='Explicit debug mode without renderer; NOT a visual experiment')
    episode = sub.add_parser('episode', help='Run one pure-RGB strategy episode')
    episode.add_argument('--policy', required=True, choices=[
        'no-extra-observation', 'self-observation', 'immediate-helper',
        'helper-after-current-B-step', 'pre-observation',
        'helper-motion-without-helper-image'])
    episode.add_argument('--b-load', type=int, choices=[1, 3, 5], default=1)
    episode.add_argument('--seed', type=int, default=0)
    episode.add_argument('--output', type=Path, required=True)
    episode.add_argument('--video', action='store_true',
                         help='Save evaluation-only overview.mp4')
    episode.add_argument('--video-stride', type=int, default=100,
                         help='Periodic video frame interval in physics steps')
    batch = sub.add_parser('batch', help='Run paired seeds, policies and Task-B loads')
    batch.add_argument('--seeds', default='100:120', help='inclusive:exclusive')
    batch.add_argument('--b-loads', default='1,3,5')
    batch.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.command == 'probe':
        args.output.mkdir(parents=True, exist_ok=False)
        import sapien
        result = dict(render_available=False)
        try:
            sapien.render.set_camera_shader_dir('rt')
            sapien.render.set_ray_tracing_samples_per_pixel(1)
            sapien.render.set_ray_tracing_path_depth(1)
            sapien.render.set_ray_tracing_denoiser('none')
            scene = sapien.Scene([sapien.physx.PhysxCpuSystem(), sapien.render.RenderSystem()])
            camera = scene.add_camera('probe', 64, 64, 1., .015, 5)
            scene.update_render()
            camera.take_picture()
            result.update(render_available=True, rgb_shape=list(camera.get_picture('Color').shape),
                          observation_modality='rgb_only')
        except Exception as exc:
            result['error'] = f'{type(exc).__name__}: {exc}'
        (args.output/'environment_probe.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result))
        return 0 if result['render_available'] else 1
    from envs.active_observation_tray.configs import Config
    if args.command == 'oracle':
        from envs.active_observation_tray.debug_oracle import run
        result = run(Config(), args.seed, args.output, args.physics_only)
        return 0 if result['success'] else 1
    if args.command == 'episode':
        from envs.active_observation_tray.experiment import run_episode
        result = run_episode(Config(), args.seed, args.policy, args.b_load, args.output,
                             args.video, args.video_stride)
        return 0 if result['success'] else 1
    from envs.active_observation_tray.experiment import run_batch
    start, stop = map(int, args.seeds.split(':'))
    loads = [int(value) for value in args.b_loads.split(',')]
    run_batch(Config(), range(start, stop), loads, args.output)
    return 0


if __name__ == '__main__':
    sys.exit(main())
