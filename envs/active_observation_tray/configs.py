"""Public geometry and dynamics. Random instance values live in the scene."""
from dataclasses import dataclass, asdict


@dataclass(frozen=True)
class Config:
    dt: float = .004
    joint_velocity: float = .8
    joint_acceleration: float = 1.6
    cartesian_velocity: float = .12
    cartesian_acceleration: float = .3
    rack_center: tuple = (-.18, .12, .80)
    tray_start: tuple = (-.18, -.25, .79)
    tray_half_width: float = .045
    peg_offset: tuple = (0., .20, .014)
    peg_half: tuple = (.005, .012, .006)
    slot_half_width: float = .011
    lateral_range: float = .014
    rack_xy_range: float = .002
    tray_xy_range: float = .002
    image_width: int = 640
    image_height: int = 480
    rt_samples_per_pixel: int = 8
    rt_bounces: int = 3
    marker_min_pixels: int = 12
    marker_max_pixels: int = 1000
    max_rgb_correction: float = .025
    final_insertion_distance: float = .065
    max_stability_seconds: float = 2.
    helper_camera_position: tuple = (.16, -.10, 1.02)
    # Bounded self-motion used to test visibility while retaining the grasp.
    # On this embodiment the held tray still occludes the rack in the real RGB.
    self_view_offset: tuple = (-.04, .02, .23)
    b_block_starts: tuple = ((.08, -.20, .76), (.14, -.20, .76),
                             (.20, -.20, .76), (.26, -.20, .76),
                             (.32, -.20, .76))
    b_block_goals: tuple = ((.08, .05, .76), (.14, .05, .76),
                            (.20, .05, .76), (.26, .05, .76),
                            (.32, .05, .76))

    def motion_config(self):
        # Legacy kinematics constructor accepts a nominal socket center.
        return {**asdict(self), 'socket_center': self.rack_center}
