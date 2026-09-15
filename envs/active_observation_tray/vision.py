"""Pure-RGB observation boundary and marker-based lateral estimator."""
from dataclasses import dataclass
from itertools import combinations, permutations

import cv2
import numpy as np

@dataclass(frozen=True)
class RGBFrame:
    name: str
    timestamp: float
    rgb: np.ndarray
    intrinsic: np.ndarray
    T_world_camera: np.ndarray


@dataclass(frozen=True)
class MarkerEstimate:
    lateral_error: float
    slot_x: float
    peg_x: float
    slot_pixels: int
    peg_pixels: int
    valid: bool
    reason: str = ''

    def to_dict(self):
        return {
            'lateral_error_rgb': self.lateral_error,
            'slot_x_rgb': self.slot_x,
            'peg_x_rgb': self.peg_x,
            'slot_pixels': self.slot_pixels,
            'peg_pixels': self.peg_pixels,
            'valid': self.valid,
            'reason': self.reason,
        }


class RGBObservationAdapter:
    """Expose pixels and calibration only; never expose the scene or actors."""

    def __init__(self, cameras, render):
        self._cameras = dict(cameras)
        self._render = render

    def capture(self, name, timestamp):
        self._render()
        camera = self._cameras[name]
        camera.take_picture()
        rgb = (camera.get_picture('Color')[..., :3] * 255).clip(0, 255).astype(np.uint8)
        world_to_camera = np.eye(4)
        world_to_camera[:3, :] = camera.get_extrinsic_matrix().copy()
        return RGBFrame(
            name=name,
            timestamp=float(timestamp),
            rgb=rgb,
            intrinsic=camera.get_intrinsic_matrix().copy(),
            T_world_camera=np.linalg.inv(world_to_camera),
        )


class ColorMarkerEstimator:
    """Recover lateral peg/slot alignment from color and calibrated rays.

    Both targets lie on public, known-height planes. No depth image, actor pose,
    instance seed, segmentation buffer or simulator geometry enters this class.
    """

    def __init__(self, rack_center, peg_offset, peg_half, min_pixels=24,
                 max_pixels=1000):
        rack_center = np.asarray(rack_center, dtype=float)
        peg_offset = np.asarray(peg_offset, dtype=float)
        self.peg_offset = peg_offset
        self.slot_prior = np.eye(4)
        self.slot_prior[:3, 3] = rack_center
        self.slot_points = np.array([[x, -.0082, z]
                                     for x in (-.022, .022)
                                     for z in (.010, .035)], dtype=np.float32)
        self.tray_points = np.array([[x, y, .0073]
                                     for x in (-.030, .030)
                                     for y in (.125, .175)], dtype=np.float32)
        self.min_pixels = int(min_pixels)
        self.max_pixels = int(max_pixels)
        self.diagnostics = {}

    @staticmethod
    def _masks(rgb):
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        red = (((hsv[..., 0] <= 4) | (hsv[..., 0] >= 174))
               & (hsv[..., 1] >= 90) & (hsv[..., 2] >= 55))
        colored = [
            ((hsv[..., 0] >= 38) & (hsv[..., 0] <= 82)),   # green
            ((hsv[..., 0] >= 5) & (hsv[..., 0] <= 21)),    # orange
            ((hsv[..., 0] >= 138) & (hsv[..., 0] <= 169)), # magenta
            ((hsv[..., 0] >= 22) & (hsv[..., 0] <= 37)),   # yellow
        ]
        colored = [mask & (hsv[..., 1] >= 75) & (hsv[..., 2] >= 45)
                   for mask in colored]
        kernel = np.ones((3, 3), np.uint8)
        clean = lambda mask: cv2.morphologyEx(  # noqa: E731
            mask.astype(np.uint8), cv2.MORPH_OPEN, kernel)
        return clean(red), tuple(clean(mask) for mask in colored)

    def _components(self, mask, bounded=False):
        count, _, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
        components = [(int(stats[i, cv2.CC_STAT_AREA]), centroids[i])
                      for i in range(1, count)
                      if stats[i, cv2.CC_STAT_AREA] >= self.min_pixels
                      and (not bounded or
                           stats[i, cv2.CC_STAT_AREA] <= self.max_pixels)]
        return sorted(components, reverse=True, key=lambda value: value[0])

    @staticmethod
    def _pose_distance(a, b):
        translation = np.linalg.norm(a[:3, 3] - b[:3, 3])
        cosine = np.clip((np.trace(a[:3, :3].T @ b[:3, :3]) - 1.) / 2., -1., 1.)
        return translation, np.arccos(cosine)

    def _solve_board(self, frame, components, object_points, prior):
        if len(components) < 4:
            return None
        best = None
        # Specular surfaces can create an extra saturated component. Test small
        # candidate subsets and let calibration plus the public nominal pose
        # select the geometrically consistent board.
        candidates = [centroid for _, centroid in components[:8]]
        for selected in combinations(candidates, 4):
            for ordered in permutations(selected):
                image_points = np.asarray(ordered, dtype=np.float32)
                ok, rotation, translation = cv2.solvePnP(
                    object_points, image_points, frame.intrinsic, None,
                    flags=cv2.SOLVEPNP_ITERATIVE)
                if not ok or translation[2, 0] <= 0:
                    continue
                camera_from_object = np.eye(4)
                camera_from_object[:3, :3] = cv2.Rodrigues(rotation)[0]
                camera_from_object[:3, 3] = translation[:, 0]
                world_from_object = frame.T_world_camera @ camera_from_object
                projected, _ = cv2.projectPoints(object_points, rotation, translation,
                                                  frame.intrinsic, None)
                reprojection = float(np.mean(np.linalg.norm(
                    projected[:, 0] - image_points, axis=1)))
                position_error, angle_error = self._pose_distance(world_from_object, prior)
                score = reprojection + 1000. * position_error + 5. * angle_error
                if best is None or score < best[0]:
                    best = score, world_from_object
        return None if best is None else best[1]

    @staticmethod
    def _project_prior(frame, prior, point):
        world = prior @ np.r_[point, 1.]
        camera = np.linalg.inv(frame.T_world_camera) @ world
        if camera[2] <= 0:
            return None
        pixel = frame.intrinsic @ camera[:3]
        return pixel[:2] / pixel[2]

    def _solve_corresponded_board(self, frame, image_points, object_points, prior):
        camera_from_prior = np.linalg.inv(frame.T_world_camera) @ prior
        image_points = np.asarray(image_points, dtype=np.float32)
        candidates = []
        ok, rotations, translations, _ = cv2.solvePnPGeneric(
            object_points, image_points, frame.intrinsic, None,
            flags=cv2.SOLVEPNP_IPPE)
        if ok:
            candidates.extend(zip(rotations, translations))
        initial_rotation, _ = cv2.Rodrigues(camera_from_prior[:3, :3])
        initial_translation = camera_from_prior[:3, 3].reshape(3, 1).copy()
        ok, rotation, translation = cv2.solvePnP(
            object_points, image_points, frame.intrinsic, None,
            initial_rotation, initial_translation, True,
            flags=cv2.SOLVEPNP_ITERATIVE)
        if ok:
            candidates.append((rotation, translation))
        best = None
        for rotation, translation in candidates:
            if translation[2, 0] <= 0:
                continue
            camera_from_object = np.eye(4)
            camera_from_object[:3, :3] = cv2.Rodrigues(rotation)[0]
            camera_from_object[:3, 3] = translation[:, 0]
            world_from_object = frame.T_world_camera @ camera_from_object
            projected, _ = cv2.projectPoints(
                object_points, rotation, translation, frame.intrinsic, None)
            reprojection = float(np.mean(np.linalg.norm(
                projected[:, 0] - image_points, axis=1)))
            position_error, angle_error = self._pose_distance(world_from_object, prior)
            score = reprojection + 1000. * position_error + 5. * angle_error
            if best is None or score < best[0]:
                best = score, world_from_object
        return None if best is None else best[1]

    def detect_slot(self, frame):
        red, _ = self._masks(frame.rgb)
        components = self._components(red)
        pose = self._solve_board(frame, components, self.slot_points, self.slot_prior)
        if pose is None:
            return None, int(red.sum()), 'fewer than four red slot markers'
        self.diagnostics['slot_pose_rgb'] = pose.tolist()
        return float(pose[0, 3]), sum(area for area, _ in components[:4]), ''

    def detect_peg(self, frame, tray_prior):
        _, color_masks = self._masks(frame.rgb)
        image_points = []
        expected_points = []
        areas = []
        for point, mask in zip(self.tray_points, color_masks):
            components = self._components(mask, bounded=True)
            expected = self._project_prior(frame, tray_prior, point)
            if not components or expected is None:
                return None, sum(areas), 'one or more tray colors unavailable'
            area, centroid = min(components,
                                 key=lambda value: np.linalg.norm(value[1] - expected))
            image_points.append(centroid)
            expected_points.append(expected)
            areas.append(area)
        pose = self._solve_corresponded_board(
            frame, image_points, self.tray_points, tray_prior)
        if pose is None:
            return None, sum(areas), 'colored tray pose unavailable'
        peg = pose[:3, :3] @ self.peg_offset + pose[:3, 3]
        self.diagnostics['tray_pose_rgb'] = pose.tolist()
        self.diagnostics['tray_prior'] = tray_prior.tolist()
        self.diagnostics['tray_marker_pixels'] = np.asarray(image_points).tolist()
        self.diagnostics['tray_marker_prior_pixels'] = np.asarray(expected_points).tolist()
        return float(peg[0]), sum(areas), ''

    def estimate(self, slot_frames, peg_frames, tray_priors, peg_x_offsets=None):
        if isinstance(tray_priors, np.ndarray):
            tray_priors = [tray_priors] * len(peg_frames)
        if peg_x_offsets is None:
            peg_x_offsets = [0.] * len(peg_frames)
        slots = [self.detect_slot(frame) for frame in slot_frames]
        pegs = []
        for frame, prior, offset in zip(peg_frames, tray_priors, peg_x_offsets):
            value = self.detect_peg(frame, prior)
            if value[0] is not None:
                value = (value[0] + offset, value[1], value[2])
            pegs.append(value)
        slots = [value for value in slots if value[0] is not None]
        pegs = [value for value in pegs if value[0] is not None]
        if not slots or not pegs:
            reasons = []
            if not slots:
                reasons.append('slot unavailable')
            if not pegs:
                reasons.append('peg unavailable')
            return MarkerEstimate(0., 0., 0., 0, 0, False, ', '.join(reasons))
        slot_pixels = sum(value[1] for value in slots)
        peg_pixels = sum(value[1] for value in pegs)
        slot_x = sum(value[0] * value[1] for value in slots) / slot_pixels
        peg_x = sum(value[0] * value[1] for value in pegs) / peg_pixels
        return MarkerEstimate(slot_x - peg_x, slot_x, peg_x,
                              slot_pixels, peg_pixels, True)

    def estimate_against_kinematic_peg(self, slot_frames, tray_prior):
        """Combine RGB slot localization with the public grasp/FK prior."""
        slots = [self.detect_slot(frame) for frame in slot_frames]
        slots = [value for value in slots if value[0] is not None]
        if not slots:
            return MarkerEstimate(0., 0., 0., 0, 0, False, 'slot unavailable')
        slot_pixels = sum(value[1] for value in slots)
        slot_x = sum(value[0] * value[1] for value in slots) / slot_pixels
        peg = tray_prior[:3, :3] @ self.peg_offset + tray_prior[:3, 3]
        self.diagnostics['peg_source'] = 'public_grasp_and_robot_fk'
        return MarkerEstimate(slot_x - float(peg[0]), slot_x, float(peg[0]),
                              slot_pixels, 0, True)
