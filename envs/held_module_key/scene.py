"""Privileged scene and physical initialization; hidden key state stays here."""
import numpy as np
import sapien
from scipy.spatial.transform import Rotation
from mplib.collision_detection import fcl
import mplib
import time
from envs.occluded_socket_drawer import occluded_socket_drawer, COLORS
from envs.occluded_socket.geometry import transform, T_EE_MODULE, module_to_ee, R_DOWN
from envs.occluded_socket.motion import RobotKinematics


def socket_boxes(center):
    """A real asymmetric L aperture, with 2.5 mm key clearance on each side."""
    result=[('socket_floor',np.array(center)+[0,0,-.020],[.06,.055,.004])]
    xs=[-.06,-.0185,.0055,.0185,.06]
    ys=[-.055,-.0125,.0125,.0185,.055]
    for i,(a,b) in enumerate(zip(xs[:-1],xs[1:])):
        for j,(c,d) in enumerate(zip(ys[:-1],ys[1:])):
            x,y=(a+b)/2,(c+d)/2
            hole=(-.0185<x<.0185 and -.0125<y<.0125) or (.0055<x<.0185 and .0125<y<.0185)
            if not hole:
                result.append((f'socket_deck_{i}_{j}',np.array(center)+[x,y,-.006],[(b-a)/2,(d-c)/2,.006]))
    return result


def rack_boxes(center):
    # Edge supports leave the actual key face exposed to a low side camera.
    p=np.array(center)
    half_height=(p[2]-.74)/2
    return [(f'rack_{i}',p+[0,y,-half_height],[.052,.003,half_height]) for i,y in enumerate([-.029,.029])]


class HeldKeyScene(occluded_socket_drawer):
    def load_actors(self):
        c=self.config
        b=self.scene.create_actor_builder()
        for _,p,half in socket_boxes(c['socket_center']):
            self.box(b,p,half,[.67,.69,.71])
        self.socket=b.build_static('key_socket')
        self.socket_xyz=np.array(c['socket_center'])
        rng=np.random.default_rng(self.seed)
        self.T_module_key=transform([*rng.uniform(-np.array(c['hidden_key_xy_range']),c['hidden_key_xy_range']),-.008],
                                  Rotation.from_euler('z',rng.uniform(*c['hidden_key_yaw_degrees']),degrees=True).as_matrix())
        b=self.scene.create_actor_builder()
        self.box(b,[0,0,0],c['module_half_size'],[.07,.28,.36])
        self.box(b,[0,0,.04],c['handle_half_size'],[.9,.6,.16])
        def key_box(p,half,color,collision=True):
            pose=sapien.Pose(self.T_module_key@transform(p))
            if collision:b.add_box_collision(pose=pose,half_size=half,material=self.material,density=300)
            b.add_box_visual(pose=pose,half_size=half,material=color)
        key_box([0,0,-.006],[.016,.010,.006],[.74,.75,.77])
        key_box([.012,.013,-.006],[.004,.003,.006],[.74,.75,.77])
        for p,color in zip(c['key_marker_points'],COLORS):
            # Cosmetic, sub-mm paint does not create the key/slot constraint.
            key_box(p,[c['key_marker_half_size']]*2+[.0001],color,collision=False)
        self.module=b.build('key_module')
        self.module_body=self.module.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent)
        self.module_body.set_mass(.08)
        b=self.scene.create_actor_builder()
        for _,p,half in rack_boxes(c['rack_center']):self.box(b,p,half,[.4,.37,.3])
        self.rack=b.build_static('edge_support')
        # Below-face lighting only; no visibility/material/collision special cases.
        self.scene.add_point_light([.0,-.25,.84],[.45,.45,.45])

    def snapshot(self):
        s=super().snapshot()
        s.update(T_module_key=self.T_module_key,
                 T_world_key=self.module.get_pose().to_transformation_matrix()@self.T_module_key,
                 module_angular_velocity=self.module_body.angular_velocity)
        return s


class KeyKinematics(RobotKinematics):
    def __init__(self,robot,config):
        super().__init__(robot,config)
        world=self.collision_planner.planning_world
        for name in ['housing_floor','housing_right','housing_back']:world.remove_object(name)
        for name,p,half in socket_boxes(config['socket_center'])+rack_boxes(config['rack_center']):
            world.add_object(name,fcl.CollisionObject(fcl.Box(np.array(half)*2),mplib.Pose(p,[1,0,0,0])))
        self.explicit_goals={}

    def path_to_joint_goal(self,arm,end,qpos=None):
        start=self.entity.get_qpos() if qpos is None else qpos.copy()
        delta=np.asarray(end)-start[self.ids[arm]]
        duration=max(1.875*np.max(abs(delta))/self.config['joint_velocity'],
                     np.sqrt(5.774*np.max(abs(delta))/self.config['joint_acceleration']),.08)
        n=int(np.ceil(duration/self.config['dt']))
        u=np.linspace(0,1,n+1)
        return start[self.ids[arm]]+(10*u**3-15*u**4+6*u**5)[:,None]*delta,((30*u**2-60*u**3+30*u**4)/(n*self.config['dt']))[:,None]*delta,0.

    def joint_path(self,arm,goal,qpos=None):
        key=tuple(np.round(goal,6).ravel())
        if key in self.explicit_goals:return self.path_to_joint_goal(arm,self.explicit_goals[key],qpos)
        return super().joint_path(arm,goal,qpos)


def initialize(env,kin):
    q=kin.entity.get_qpos()
    goals={'right':module_to_ee(transform(env.config['initial_module_center'])),
           'left':transform([-.25,-.25,1.14],R_DOWN)}
    for arm,goal in goals.items():q=kin.ik(arm,goal,q)
    for arm in ['left','right']:
        ids=[kin.entity.get_active_joints().index(j[0]) for j in getattr(env.robot,arm+'_gripper')]
        q[ids]=.016 if arm=='right' else 0
    kin.entity.set_qpos(q)
    for arm in goals:
        kin.drive(arm,q[kin.ids[arm]]);kin.gripper(arm,0)
    env.module.set_pose(sapien.Pose(goals['right']@T_EE_MODULE))
    checkpoint_q=q.copy()
    initial_hits=kin.check_path('right',np.array([q[kin.ids['right']]]))
    if initial_hits:
        return dict(initialization_seconds=0.,valid=False,initial_robot_collisions=initial_hits,
                    designed_T_ee_module=T_EE_MODULE,checkpoint_q=checkpoint_q)
    history=[]
    for i in range(750):
        kin.compensate();env.step_physics()
        if i%25==0:
            history.append(dict(seconds=(i+1)*env.config['dt'],T_ee_module=np.linalg.inv(kin.fk('right'))@env.module.get_pose().to_transformation_matrix(),
                                qpos=kin.entity.get_qpos(),linear_velocity=env.module_body.linear_velocity,
                                angular_velocity=env.module_body.angular_velocity,
                                contacts=[dict(bodies=[b.entity.name for b in c.bodies],impulse=float(sum(np.linalg.norm(p.impulse) for p in c.points))) for c in env.scene.get_contacts()]))
    relative=np.linalg.inv(kin.fk('right'))@env.module.get_pose().to_transformation_matrix()
    kin.reference_qpos=kin.entity.get_qpos().copy()
    return dict(initialization_seconds=3.,designed_T_ee_module=T_EE_MODULE,measured_T_ee_module_gt=relative,
                checkpoint_q=checkpoint_q,initial_robot_collisions=initial_hits,
                history=history,valid=bool(np.linalg.norm(relative[:3,3]-T_EE_MODULE[:3,3])<.025 and not initial_hits))
