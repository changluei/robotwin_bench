"""Local adapter of existing robot-only time-parameterized trajectories."""
from envs.occluded_socket.motion import RobotKinematics


class TrayKinematics(RobotKinematics):
    def __init__(self, robot, config):
        super().__init__(robot, config.motion_config())
        # Remove unrelated historical socket fixtures. Actual new fixture
        # contacts remain enabled in PhysX. Phase 2 records contact diagnostics;
        # a public rack collision planner is required before visual baselines.
        for name in ['housing_floor', 'housing_right', 'housing_back']:
            self.collision_planner.planning_world.remove_object(name)
