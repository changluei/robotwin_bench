"""Public geometry/calibration only. T_A_B maps B coordinates into A (metres)."""
import numpy as np
from scipy.spatial.transform import Rotation


def transform(position=(0, 0, 0), rotation=None):
    result = np.eye(4)
    result[:3, 3] = position
    if rotation is not None:
        result[:3, :3] = rotation
    return result


def look_at(position, target):
    forward = np.asarray(target) - position
    forward = forward / np.linalg.norm(forward)
    left = np.cross([0, 0, 1], forward)
    left /= np.linalg.norm(left)
    return transform(position, np.column_stack([forward, left, np.cross(forward, left)]))


# SAPIEN camera: forward/left/up; OpenCV: right/down/forward.
T_SAPIEN_CV = transform(rotation=np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]))
# Exact public URDF fixed transform fr_link6 -> right_camera (same on left).
T_EE_CAMERA = transform([0.07, 0.032, 0.065], Rotation.from_euler('y', 0.4).as_matrix())
R_DOWN = np.array([[0., 0, 1], [0, 1, 0], [-1, 0, 0]])
# Module center is 12 cm along the gripper axis plus 4 cm to its bottom plate.
# This is a designed grasp calibration, never refreshed from actor state.
T_EE_MODULE = transform([0.16, 0, 0], R_DOWN.T)


def module_to_ee(T_world_module):
    return T_world_module @ np.linalg.inv(T_EE_MODULE)
