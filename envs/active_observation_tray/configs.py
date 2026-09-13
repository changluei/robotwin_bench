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
    slot_half_width: float = .010
    lateral_range: float = .014
    rack_xy_range: float = .002
    tray_xy_range: float = .002

    def motion_config(self):
        # Legacy kinematics constructor accepts a nominal socket center.
        return {**asdict(self), 'socket_center': self.rack_center}
