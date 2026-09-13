"""PHASE 2 ONLY. Deliberate GT input; never import from a tested controller."""
import hashlib
import json
import traceback
from dataclasses import asdict
from pathlib import Path
import numpy as np
from envs.occluded_socket.geometry import transform, R_DOWN
from .scene import TrayScene
from .motion import TrayKinematics
from .evaluation import measure_geometry


def run(config, seed, output, physics_only=False):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    result = dict(mode='PHASE_2_ORACLE_GEOMETRY_ONLY', seed=seed,
                  eligible_for_strategy_benchmark=False, success=False,
                  camera_validation=False, physics_only=physics_only)
    (output / 'config.json').write_text(json.dumps(asdict(config), indent=2))
    root = Path(__file__).resolve().parents[2]
    files = [*Path(__file__).parent.glob('*.py'), root/'scripts/active_observation_tray_demo.py',
             root/'envs/occluded_socket/motion.py', root/'envs/occluded_socket/geometry.py',
             root/'envs/robot/robot.py']
    provenance = {}
    for file in files:
        relative = file.relative_to(root)
        dest = output / 'source' / relative
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes(file.read_bytes())
        provenance[str(relative)] = hashlib.sha256(file.read_bytes()).hexdigest()
    (output / 'source_sha256.json').write_text(json.dumps(provenance, indent=2))
    env = TrayScene()
    log = (output / 'trajectory_gt.jsonl').open('w')
    actions = []
    try:
        env.setup_demo(config, seed, physics_only)
        kin = TrayKinematics(env.robot, config)
        q = kin.entity.get_qpos().copy()
        for arm, p in [('left', [-.18, -.27, 1.12]), ('right', [.25, -.25, 1.14])]:
            q = kin.ik(arm, transform(p, R_DOWN), q)
            for joint, _, _ in getattr(env.robot, arm+'_gripper'):
                q[kin.entity.get_active_joints().index(joint)] = .04
        kin.entity.set_qpos(q)  # Initial configuration only, before episode t=0.
        for arm in ['left', 'right']:
            kin.drive(arm, q[kin.ids[arm]])
            kin.gripper(arm, .04)
        initial = dict(qpos=q.tolist(), tray=env.tray.get_pose().p.tolist(), hole=env.hole_xyz.tolist())
        (output/'initial_state_gt.json').write_text(json.dumps(initial, indent=2))
        result['initial_state_sha256'] = hashlib.sha256(json.dumps(initial, sort_keys=True).encode()).hexdigest()

        def tick(action):
            kin.compensate()
            env.step_physics()
            if env.steps % 10 == 0:
                contacts = [dict(bodies=[b.entity.name for b in ct.bodies],
                                 impulse=float(sum(np.linalg.norm(p.impulse) for p in ct.points)))
                            for ct in env.scene.get_contacts()
                            if any(b.entity.name == 'tray' for b in ct.bodies)]
                log.write(json.dumps(dict(step=env.steps, sim_time=env.steps*config.dt,
                                          action=action, qpos=kin.entity.get_qpos().tolist(),
                                          contacts_gt=contacts, **measure_geometry(env)))+'\n')

        def move(name, goal):
            path, velocity, _ = kin.cartesian_path('left', goal)
            start = env.steps
            for p, v in zip(path, velocity):
                kin.drive('left', p, v)
                tick(name)
            state = measure_geometry(env)
            actions.append(dict(name=name, start_step=start, end_step=env.steps,
                                seconds=(env.steps-start)*config.dt, **state))
            print(name, 'step', env.steps, 'error', state['lateral_error_gt'],
                  state['insertion_error_gt'], state['vertical_error_gt'], flush=True)
            if not physics_only:
                env.render()
                from PIL import Image
                for camera_name, camera in env.cameras.items():
                    camera.take_picture()
                    rgb = (camera.get_picture('Color')[..., :3]*255).clip(0,255).astype('uint8')
                    Image.fromarray(rgb).save(output/f'{name}_{camera_name}.png')

        def grip(name, opening):
            start = .04 if opening == 0 else 0
            count = int(.5/config.dt)
            for i in range(count):
                kin.gripper('left', start+(opening-start)*(i+1)/count)
                tick(name)

        # Actual grasp from the source support. All GT use is confined here.
        pick = transform(env.tray_initial+[0, 0, .16], R_DOWN)
        hover = pick.copy(); hover[2,3] += .06
        move('pick_hover', hover)
        move('pick_descend', pick)
        grip('close_gripper', 0.)
        lift = pick.copy(); lift[2,3] += .025
        move('lift', lift)
        intermediate = lift.copy()
        intermediate[1,3] = config.rack_center[1] - config.peg_offset[1] - .04
        intermediate[2,3] = config.rack_center[2] + .16
        move('partial_insertion', intermediate)
        # Live GT is allowed only for this explicit geometry debugging command.
        error = measure_geometry(env)['lateral_error_gt']
        corrected = kin.fk('left'); corrected[0,3] += error
        move('oracle_lateral_correction', corrected)
        final = corrected.copy()
        final[1,3] += measure_geometry(env)['insertion_error_gt'] + .01
        move('final_insertion', final)
        # Still actively holding the handle: no release requirement in A.
        # Short stability verification is a common physical completion criterion,
        # not an observation sleep/penalty. Its ticks are included in reported time.
        stable = 0
        required_stable = int(.5/config.dt)
        for _ in range(int(2./config.dt)):
            tick('verify_hold')
            state = measure_geometry(env)
            stable = stable+1 if state['inserted'] and state['tray_linear_speed'] < .02 else 0
            if stable >= required_stable:
                break
        result.update(success=stable >= required_stable, stable_steps=stable,
                      sim_steps=env.steps, simulation_seconds=env.steps*config.dt,
                      final_geometry_gt=measure_geometry(env), actions=actions)
    except Exception as exc:
        result.update(error=f'{type(exc).__name__}: {exc}', sim_steps=env.steps,
                      simulation_seconds=env.steps*config.dt, actions=actions)
        (output/'error.log').write_text(traceback.format_exc())
        traceback.print_exc()
    finally:
        log.close()
        env.close_env()
        (output/'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps({k:v for k,v in result.items() if k not in ['actions', 'final_geometry_gt']}), flush=True)
    return result
