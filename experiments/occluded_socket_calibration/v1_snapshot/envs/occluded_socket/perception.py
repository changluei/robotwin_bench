"""RGB color detection + metric depth geometry. No environment imports."""
from dataclasses import dataclass
from itertools import product
import cv2
import numpy as np


@dataclass
class Detection:
    camera: str
    timestamp: float
    valid: bool
    reason: str
    pixels: list
    counts: list
    geometry_error: float | None = None
    reprojection_error: float | None = None
    T_world_socket: np.ndarray | None = None

    def to_dict(self):
        return {k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in vars(self).items()}


class Perception:
    def __init__(self, marker_points, thresholds):
        self.points = np.array(marker_points, dtype=float)
        self.thresholds = thresholds.copy()
        self.previous = {}
        self.streak = {}

    def detect(self, frame):
        hsv = cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2HSV)
        h, s, v = cv2.split(hsv)
        masks = [(h < 9) | (h > 172), (h > 42) & (h < 82),
                 (h > 102) & (h < 132), (h > 20) & (h < 38)]
        pixels, counts, points_cam = [], [], []
        alternatives=[]
        for mask in masks:
            mask = (mask & (s > 130) & (v > 70)).astype('uint8')
            n, labels, stats, centers = cv2.connectedComponentsWithStats(mask)
            candidates = [i for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= self.thresholds['min_pixels']]
            components=[]
            for idx in candidates:
                uv=centers[idx]
                z=float(np.median(frame.depth[labels==idx]))
                width=max(stats[idx,cv2.CC_STAT_WIDTH],stats[idx,cv2.CC_STAT_HEIGHT])*z/frame.intrinsic[0,0]
                if .02<z<2 and .003<width<.028:
                    xyz=np.linalg.inv(frame.intrinsic)@[*uv,1]*z
                    components.append((uv.tolist(),int(stats[idx,cv2.CC_STAT_AREA]),xyz))
            components=sorted(components,key=lambda c:-c[1])[:6]
            alternatives.append(components)
            if not components:
                pixels.append(None); counts.append(0)
                continue
            pixels.append(components[0][0]);counts.append(components[0][1]);points_cam.append(components[0][2])
        result = Detection(frame.name, frame.timestamp, False, 'incomplete_markers', pixels, counts)
        if len(points_cam) != 4:
            self.streak[frame.name] = 0
            return result
        def fit_error(combination):
            world=np.array([v[2] for v in combination])@frame.T_world_camera[:3,:3].T+frame.T_world_camera[:3,3]
            offsets=world-self.points
            return np.max(np.linalg.norm(offsets-offsets.mean(axis=0),axis=1))
        chosen=min(product(*alternatives),key=fit_error)
        pixels=[v[0] for v in chosen];counts=[v[1] for v in chosen];points_cam=[v[2] for v in chosen]
        result.pixels,result.counts=pixels,counts
        points_cam = np.array(points_cam)
        # Translation-only experiment: known upright fixture rotation. Fit translation
        # in world space and validate all four measured pairwise relations.
        points_world = points_cam @ frame.T_world_camera[:3, :3].T + frame.T_world_camera[:3, 3]
        xyz = np.mean(points_world - self.points, axis=0)
        error = float(np.max(np.linalg.norm(points_world - self.points - xyz, axis=1)))
        T = np.eye(4); T[:3, 3] = xyz
        predicted_world = self.points + xyz
        inverse = np.linalg.inv(frame.T_world_camera)
        predicted_cam = predicted_world @ inverse[:3,:3].T + inverse[:3,3]
        projected = predicted_cam @ frame.intrinsic.T
        projected = projected[:,:2] / projected[:,2:]
        reprojection = float(np.sqrt(np.mean(np.sum((projected - pixels)**2,axis=1))))
        result.geometry_error, result.reprojection_error = error, reprojection
        result.T_world_socket = T
        if error > self.thresholds['max_geometry_error'] or reprojection > self.thresholds['max_reprojection_error']:
            result.reason = 'geometry_or_reprojection'
            self.streak[frame.name] = 0
            return result
        previous = self.previous.get(frame.name)
        consistent = previous is not None and np.linalg.norm(previous - xyz) <= self.thresholds['max_temporal_translation']
        self.streak[frame.name] = self.streak.get(frame.name, 0) + 1 if consistent else 1
        self.previous[frame.name] = xyz
        result.valid = self.streak[frame.name] >= self.thresholds['consistent_frames']
        result.reason = 'valid' if result.valid else 'await_consistency'
        return result
