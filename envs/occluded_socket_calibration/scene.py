"""Optional v2 physical occlusion; original v1 scene remains unchanged."""
import numpy as np
import mplib
from mplib.collision_detection import fcl
from envs.occluded_socket_drawer import occluded_socket_drawer
from envs.occluded_socket.motion import RobotKinematics


class OcclusionChannelScene(occluded_socket_drawer):
    def load_actors(self):
        super().load_actors()
        b=self.scene.create_actor_builder()
        wall=self.config['front_occlusion_wall']
        self.box(b,wall['center'],wall['half_size'],[.22,.24,.28])
        self.front_wall=b.build_static('fixed_front_occlusion_wall')


class ChannelKinematics(RobotKinematics):
    def __init__(self,robot,config):
        super().__init__(robot,config)
        # Same public fixed wall in the planning and physical worlds.
        wall=config['front_occlusion_wall']
        self.collision_planner.planning_world.add_object('fixed_front_occlusion_wall',
            fcl.CollisionObject(fcl.Box(np.array(wall['half_size'])*2),mplib.Pose(wall['center'],[1,0,0,0])))
