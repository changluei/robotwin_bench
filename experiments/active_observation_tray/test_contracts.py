"""CPU-only contracts for the pure-RGB experiment boundary."""
import ast
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from envs.active_observation_tray.vision import (  # noqa: E402
    ColorMarkerEstimator,
    RGBFrame,
)


class Contracts(unittest.TestCase):
    def test_rgb_marker_pose_estimator(self):
        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        intrinsic = np.array([[500., 0, 320.], [0, 500., 240.], [0, 0, 1.]])
        camera = np.eye(4)
        frame = RGBFrame('synthetic', 0., rgb, intrinsic, camera)
        estimator = ColorMarkerEstimator((-.12, .12, .8), (0., .2, .014),
                                         (.005, .012, .006), 20)
        tray_pose = np.eye(4)
        tray_pose[:3, 3] = [.08, .10, .8]

        def project(pose, points):
            camera_points = (pose[:3, :3] @ points.T + pose[:3, 3:4]).T
            pixels = (intrinsic @ camera_points.T).T
            return np.rint(pixels[:, :2] / pixels[:, 2:3]).astype(int)

        for point in ((250, 220), (250, 260), (390, 220), (390, 260)):
            cv2.circle(rgb, tuple(point), 7, (255, 0, 0), -1)
        tray_colors = [(0, 255, 0), (255, 64, 0),
                       (255, 0, 255), (255, 255, 0)]
        for point, color in zip(project(tray_pose, estimator.tray_points), tray_colors):
            cv2.circle(rgb, tuple(point), 7, color, -1)
        estimate = estimator.estimate([frame], [frame], tray_pose)
        self.assertTrue(estimate.valid)
        self.assertTrue(np.isfinite(estimate.lateral_error))

    def test_policy_boundary_has_no_depth_or_privileged_pose(self):
        paths = [ROOT / 'envs/active_observation_tray/vision.py',
                 ROOT / 'envs/active_observation_tray/experiment.py']
        forbidden_attributes = {'get_pose', 'hole_xyz', 'tray_pose_gt', 'seed'}
        for path in paths:
            source = path.read_text()
            self.assertNotIn("get_picture('Position')", source)
            tree = ast.parse(source)
            if path.name == 'vision.py':
                attributes = {node.attr for node in ast.walk(tree)
                              if isinstance(node, ast.Attribute)}
                self.assertFalse(attributes & forbidden_attributes)

    def test_every_policy_is_exposed_by_cli(self):
        source = (ROOT / 'scripts/active_observation_tray_demo.py').read_text()
        for policy in ('no-extra-observation', 'self-observation',
                       'immediate-helper', 'helper-after-current-B-step',
                       'pre-observation', 'helper-motion-without-helper-image'):
            self.assertIn(policy, source)


if __name__ == '__main__':
    unittest.main()
