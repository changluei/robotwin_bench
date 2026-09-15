"""Privileged measurements. Never feed this output to a visual controller."""
import numpy as np


def measure_geometry(env):
    T = env.tray.get_pose().to_transformation_matrix()
    peg = T[:3, :3] @ env.config.peg_offset + T[:3, 3]
    error = env.hole_xyz - peg
    return dict(peg_world=peg.tolist(), hole_world=env.hole_xyz.tolist(),
                lateral_error_gt=float(error[0]), insertion_error_gt=float(error[1]),
                vertical_error_gt=float(error[2]), tray_pose_gt=T.tolist(),
                tray_linear_speed=float(np.linalg.norm(env.tray_body.linear_velocity)),
                tray_angular_speed=float(np.linalg.norm(env.tray_body.angular_velocity)),
                inserted=bool(abs(error[0]) < env.config.slot_half_width-env.config.peg_half[0]
                              and error[1] < -.005 and error[1] > -.04
                              and abs(error[2]) < .009))


def measure_task_b(env, load):
    errors = [float(np.linalg.norm(block.get_pose().p - goal))
              for block, goal in zip(env.b_blocks[:load], env.b_goal_positions[:load])]
    return dict(task_b_errors_gt=errors,
                task_b_completed_gt=[error < .035 for error in errors],
                task_b_success=bool(errors and all(error < .035 for error in errors)))
