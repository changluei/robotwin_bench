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
                              and error[1] < -.005 and error[1] > -.02
                              and abs(error[2]) < .009))
