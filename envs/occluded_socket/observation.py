"""Allow-list adapter. Images, calibrated camera transforms and robot state only."""
from dataclasses import dataclass
import numpy as np
from .geometry import T_EE_CAMERA, T_SAPIEN_CV


@dataclass(frozen=True)
class CameraFrame:
    name: str
    timestamp: float
    rgb: np.ndarray
    depth: np.ndarray
    intrinsic: np.ndarray
    T_world_camera: np.ndarray


class ObservationAdapter:
    def __init__(self, wrist_cameras, head_camera, kinematics):
        # No environment, seed, actor or evaluator reference.
        self.wrist_cameras = wrist_cameras
        self.head_camera = head_camera
        self.kinematics = kinematics
        self.head_T = head_camera.get_pose().to_transformation_matrix() @ T_SAPIEN_CV

    def capture(self, timestamp):
        frames = {}
        for arm, camera in self.wrist_cameras.items():
            T = self.kinematics.fk(arm) @ T_EE_CAMERA @ T_SAPIEN_CV
            frames[arm] = self._frame(camera, arm, timestamp, T)
        frames['head'] = self._frame(self.head_camera, 'head', timestamp, self.head_T)
        return frames

    @staticmethod
    def _frame(camera, name, timestamp, T):
        camera.take_picture()
        rgb = (camera.get_picture('Color')[..., :3] * 255).clip(0, 255).astype(np.uint8)
        # Only scalar optical depth, never GT xyz/segmentation/visibility.
        depth = -camera.get_picture('Position')[..., 2].copy()
        return CameraFrame(name, timestamp, rgb, depth, camera.get_intrinsic_matrix().copy(), T.copy())
