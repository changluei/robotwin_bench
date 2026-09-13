"""Bounded public-geometry route comparisons with shared v1 perception/control."""
import numpy as np
from scipy.spatial.transform import Rotation
from envs.occluded_socket.controller import Controller
from envs.occluded_socket.geometry import transform,look_at,T_EE_CAMERA,module_to_ee


# Finite development candidate set. No instance seed, hidden target, or load input.
ROUTES={
    'helper_original':dict(kind='helper',camera=[-.08,-.12,1.0]),
    'helper_near':dict(kind='helper',camera=[-.12,-.18,1.04]),
    'helper_front':dict(kind='helper',camera=[.00,-.22,1.04]),
    'helper_high':dict(kind='helper',camera=[.00,-.15,1.13]),
    'helper_wrist':dict(kind='helper_delta',delta=[0,0,0],pitch=-.45,yaw=.20),
    'helper_wrist_side':dict(kind='helper_delta',delta=[.04,0,0],pitch=-.55,yaw=.25),
    'helper_low_side':dict(kind='helper',camera=[-.08,-.10,.91]),
    'helper_low_front':dict(kind='helper',camera=[-.03,-.16,.94]),
    'helper_camera_here':dict(kind='helper_camera_here'),
    'helper_wrist_turn':dict(kind='helper_delta',delta=[0,0,0],pitch=-.60,yaw=.65),
    'self_camera_here':dict(kind='self_camera_here',offset=.10),
    'self_small_x':dict(kind='self_delta',delta=[.06,0,0]),
    'self_small_yaw':dict(kind='self_delta',delta=[.03,0,0],yaw=.3),
    'self_small_back':dict(kind='self_delta',delta=[0,-.06,0]),
    'self_large_x':dict(kind='self_delta',delta=[.15,0,.03]),
    'self_mid_x08':dict(kind='self_delta',delta=[.08,0,0]),
    'self_mid_x10':dict(kind='self_delta',delta=[.10,0,0]),
    'self_mid_x12':dict(kind='self_delta',delta=[.12,0,0]),
    'self_original':dict(kind='self_camera',camera=[.35,-.28,1.07],offset=.10),
    'self_large_side':dict(kind='self_camera',camera=[.37,-.12,1.06],offset=.16),
    'direct_live':dict(kind='direct',initial_only=False),
    'direct_initial_only':dict(kind='direct',initial_only=True),
    'putdown':dict(kind='putdown',force_cycle=False),
    'putdown_cycle_validation':dict(kind='putdown',force_cycle=True),
}


class RouteController(Controller):
    def __init__(self,policy,kinematics,public_config,log,variant='baseline',route=None):
        super().__init__(policy,kinematics,public_config,log,variant)
        self.route=dict(route)
        self.kind=self.route['kind']
        self.nominal_started=False
        self.rack_phase=None
        self.rack_pose=transform([.38,-.18,.788])  # public fixed support surface + module half-height

    def observing(self,arm,t):
        if self.view_index[arm]:
            self.failure='perception_candidate_exhausted'
            self.event(t,'failure',reason=self.failure)
            return
        self.view_index[arm]+=1
        if self.kind in ['self_delta','helper_delta']:
            goal=self.kin.fk(arm).copy()
            goal[:3,3]+=np.array(self.route['delta'])
            rotation=Rotation.from_euler('zy',[self.route.get('yaw',0.),self.route.get('pitch',0.)]).as_matrix()
            goal[:3,:3]=rotation@goal[:3,:3]
        else:
            position=(self.kin.fk(arm)@T_EE_CAMERA)[:3,3] if self.kind.endswith('camera_here') else self.route.get('camera',[.35,-.28,1.07])
            camera=look_at(np.array(position),self.config['socket_center'])
            offset=self.route.get('offset',0.)
            if arm=='right' and self.kind!='putdown':
                camera=look_at(np.array(position),np.array(self.config['socket_center'])-camera[:3,1]*offset)
            goal=camera@np.linalg.inv(T_EE_CAMERA)
        self.add(arm,'observe_candidate',goal)
        self.event(t,'observation_candidate',arm=arm,public_goal=goal,route=self.route)
        self.observe_since=None

    def observe(self,t,frames,detections):
        before=self.information is not None
        accepted=[] if self.kind=='direct' and self.route['initial_only'] and t>.100001 else detections
        super().observe(t,frames,accepted)
        if not before and self.information is not None:
            active=self.active['right']
            if self.kind=='direct' and active and active['name']=='direct_preapproach':
                self.cancel_right(t,'visual_target_available')
            if self.kind=='putdown' and not self.route['force_cycle'] and self.rack_phase=='placing':
                if active and active['name'] in ['rack_preapproach','rack_lower']:
                    self.cancel_right(t,'information_available_while_still_holding')
                    self.rack_phase='held_ready'
                    self.event(t,'putdown_not_needed',reason='valid_information_before_release')

    def cancel_right(self,t,reason):
        if self.active['right'] is not None:
            self.event(t,'action_cancel',arm='right',name=self.active['right']['name'],reason=reason)
        self.active['right']=None;self.queues['right'].clear()

    def tick(self,t):
        if self.kind not in ['direct','putdown']:
            return super().tick(t)
        if not self.started:
            self.started=True
            self.drawer(t)
            if self.kind=='putdown':
                self.rack_phase='placing'
                pre=self.rack_pose.copy();pre[2,3]+=.12
                seat=self.rack_pose.copy();seat[2,3]+=.002
                retreat=self.rack_pose.copy();retreat[2,3]+=.15
                self.add('right','rack_preapproach',module_to_ee(pre))
                self.add('right','rack_lower',module_to_ee(seat),cartesian=True)
                self.add('right','rack_release',grip=.045)
                self.add('right','rack_withdraw',module_to_ee(retreat),cartesian=True)
                self.event(t,'putdown_started',validation_only=self.route['force_cycle'])
        if self.kind=='direct':
            # Both direct variants share two actual initial frames at 0/.1 s.
            # A public nominal preapproach is safe coarse motion, never a GT seat.
            if t>=.1 and not self.nominal_started and self.information is None:
                self.nominal_started=True
                goal=transform(self.config['socket_center']);goal[2,3]+=.15+self.config['module_half_size'][2]
                self.add('right','direct_preapproach',module_to_ee(goal))
                self.event(t,'direct_coarse_approach',public_goal=goal)
            if self.information is not None and not self.install_started:
                self.install(t)
        else:
            idle=self.active['right'] is None and not self.queues['right']
            if idle and self.rack_phase=='placing':
                self.rack_phase='observing'
                self.event(t,'module_on_rack_command_complete')
                if self.information is None:self.observing('right',t)
            if self.rack_phase=='observing' and self.information is not None and idle:
                self.rack_phase='recovering'
                pre=self.rack_pose.copy();pre[2,3]+=.12
                lift=self.rack_pose.copy();lift[2,3]+=.15
                self.add('right','rack_regrasp_pre',module_to_ee(pre))
                self.add('right','rack_regrasp_down',module_to_ee(self.rack_pose),cartesian=True)
                self.add('right','rack_regrasp_close',grip=0.)
                self.add('right','rack_regrasp_lift',module_to_ee(lift),cartesian=True)
                self.event(t,'regrasp_started',public_rack_pose=self.rack_pose)
            if idle and self.rack_phase=='recovering' and self.status['right']=='rack_regrasp_lift_done':
                self.rack_phase='held_ready'
                self.event(t,'held_installation_restored')
            if self.rack_phase=='held_ready' and self.information is not None and not self.install_started:
                self.install(t)
        for arm in ['left','right']:self.advance(arm,t)
        self.kin.compensate()
