"""Bounded geometry routes; all three cameras remain available to every route."""
import numpy as np
from scipy.spatial.transform import Rotation
from envs.occluded_socket.controller import Controller
from envs.occluded_socket.geometry import transform,look_at,T_EE_CAMERA,T_EE_MODULE,T_SAPIEN_CV,module_to_ee


def route_candidates(route,kin,public):
    module=kin.fk('right')@T_EE_MODULE
    center=module[:3,3]
    if route=='hold_translate':
        for i,delta in enumerate([[.08,0,0],[0,-.08,0],[0,0,.07],[-.07,0,0]]):
            goal=kin.fk('right').copy();goal[:3,3]+=delta
            yield f'translate_{i}','right',goal
    elif route=='helper':
        for x in [-.05,.02,.08]:
            for y in [-.27,-.16,-.04]:
                for z in [.82,.86,.90]:
                    camera=look_at(np.array([x,y,z]),center+[0,0,-.02])
                    for roll in [0,90,-90,180]:
                        rolled=camera@transform(rotation=Rotation.from_euler('x',roll,degrees=True).as_matrix())
                        yield f'helper_{x}_{y}_{z}_roll{roll}','left',rolled@np.linalg.inv(T_EE_CAMERA)
    elif route in ['head','passive_left']:
        camera=np.array([-.032,-.45,1.35]) if route=='head' else (kin.fk('left')@T_EE_CAMERA)[:3,3]
        normal=np.array([0.,0,-1])
        module_centers=[center+delta for delta in [[0,0,0],[0,-.06,0],[-.06,0,0],[0,0,-.06]]]
        if route=='head':
            module_centers=[np.array(p) for p in [[.18,-.25,.92],[.12,-.25,.94],[.25,-.27,.95],[.18,-.30,.98],[.10,-.20,.87]]]
        if route=='passive_left':
            mounted=kin.fk('left')@T_EE_CAMERA
            module_centers=[mounted[:3,3]+mounted[:3,0]*d for d in [.15,.20,.25]]
        for angle in [40,55,70,90,110,130,150]:
            for i,position in enumerate(module_centers):
                direction=camera-position;direction/=np.linalg.norm(direction)
                axis=np.cross(normal,direction);axis/=np.linalg.norm(axis)
                R=Rotation.from_rotvec(axis*np.deg2rad(angle)).as_matrix()
                for twist in [0,90,-90,180]:
                    rotation=R@Rotation.from_euler('z',twist,degrees=True).as_matrix()
                    yield f'{route}_{angle}_p{i}_twist{twist}','right',module_to_ee(transform(position,rotation))
    elif route=='putdown':
        goal=transform(public['rack_center']);goal[2,3]+=.008+.06
        yield 'rack_pre','right',module_to_ee(goal)
    else:raise ValueError(route)


def rank_routes(route,kin,public):
    """Robot/known-fixture feasibility only; no hidden key pose or rendered oracle."""
    rows=[]
    for name,arm,goal in route_candidates(route,kin,public):
        try:
            path,_,_=kin.joint_path(arm,goal)
            hits=kin.check_path(arm,path)
            rows.append(dict(name=name,arm=arm,goal=goal,seconds=len(path)*public['dt'],feasible=not hits,collisions=hits))
        except RuntimeError as exc:
            rows.append(dict(name=name,arm=arm,goal=goal,feasible=False,error=str(exc)))
    feasible=sorted([r for r in rows if r['feasible']],key=lambda r:r['seconds'])
    return feasible,rows


def rank_joint_presentations(route,kin,public):
    """Finite robot-joint search avoids equating one failed IK branch with infeasibility.

    Uses only robot FK, fixed camera calibration and nominal outer body geometry.
    The actual hidden bottom key is neither queried nor projected.
    """
    if route not in ['head','passive_left']:raise ValueError('joint search is for presentation paths')
    cam=look_at(np.array([-.032,-.45,1.35]),[0,0,.75]) if route=='head' else kin.fk('left')@T_EE_CAMERA
    inverse=np.linalg.inv(cam@T_SAPIEN_CV)
    rng=np.random.default_rng(71)  # planner constant, unrelated to module/instance seed
    start=kin.entity.get_qpos().copy();possible=[]
    for i in range(6000):
        delta=rng.uniform(-1,1,6)*np.array([1.6,1.,1.,3.14,3.14,3.14])
        q=start.copy();q[kin.ids['right']]+=delta
        ee=kin.fk('right',q);module=ee@T_EE_MODULE;position=module[:3,3]
        if not (-.45<position[0]<.50 and -.48<position[1]<.20 and .84<position[2]<1.15):continue
        cv=inverse@np.r_[position,1.]
        if cv[2]<.1 or abs(cv[0]/cv[2])>.60 or abs(cv[1]/cv[2])>.42:continue
        direction=cam[:3,3]-position;direction/=np.linalg.norm(direction)
        facing=float(-module[:3,2]@direction)
        if facing<.40:continue
        seconds=max(1.875*np.max(abs(delta))/.8,np.sqrt(5.774*np.max(abs(delta))/1.6))
        possible.append(dict(name=f'{route}_joint_{i}',arm='right',goal=ee,goal_qpos=q[kin.ids['right']],
                             seconds=seconds,nominal_facing_cosine=facing))
    rows=[]
    for row in sorted(possible,key=lambda x:x['seconds'])[:100]:
        path,_,_=kin.path_to_joint_goal('right',row['goal_qpos'])
        hits=kin.check_path('right',path)
        rows.append(dict(**row,feasible=not hits,collisions=hits))
    return [r for r in rows if r['feasible']],rows


class KeyController(Controller):
    def __init__(self,kin,public,log,route,candidate,install=False,cycle=False):
        super().__init__('H' if route=='helper' else 'S',kin,public,log)
        self.route=route;self.candidate=candidate;self.full_install=install;self.cycle=cycle
        if 'goal_qpos' in candidate:kin.explicit_goals[tuple(np.round(candidate['goal'],6).ravel())]=candidate['goal_qpos']
        self.initial_ee={a:kin.fk(a).copy() for a in ['left','right']}
        self.phase='new';self.held=True;self.done_since=None
        self.rack_view_done_since=None
        self.rack_module=transform(public['rack_center']);self.rack_module[2,3]+=.008
        self.recovery_started=False

    def cancel(self,arm,t):
        if self.active[arm]:self.event(t,'action_cancel',arm=arm,name=self.active[arm]['name'],reason='information_ready')
        self.active[arm]=None;self.queues[arm].clear()
        self.kin.drive(arm,self.kin.entity.get_qpos()[self.kin.ids[arm]])

    def observe(self,t,frames,detections):
        if self.information is not None:return
        valid=next((d for d in detections if d['valid']),None)
        if valid is None:return
        self.information=valid['T_ee_key'].copy() if self.held else T_EE_MODULE@np.linalg.inv(self.rack_module)@valid['T_world_key']
        self.t_information_ready=t
        self.event(t,'information_ready',camera=valid['camera'],T_ee_key_estimate=self.information,held=self.held)
        if self.route!='putdown' or (not self.cycle and self.phase=='placing' and self.active['right'] and self.active['right']['name'] in ['rack_pre','rack_lower']):
            self.cancel(self.candidate['arm'],t)
            self.phase='observed'

    def recover(self,t):
        self.recovery_started=True
        if self.route=='helper':self.add('left','helper_restore',self.initial_ee['left'])
        if self.full_install and self.information is not None:
            self.install_started=True
            ee=transform(self.config['socket_center'])@np.linalg.inv(self.information)
            pre=ee.copy();pre[2,3]+=.12
            self.pre_goal=pre.copy()
            seat=ee.copy();seat[2,3]+=.0005
            retreat=seat.copy();retreat[2,3]+=.13
            self.add('right','install_preapproach',pre)
            self.add('right','install_insert',seat,cartesian=True)
            self.add('right','install_release',grip=.045)
            self.add('right','install_retreat',retreat,cartesian=True)
            self.event(t,'installation_target',T_world_ee=seat,T_ee_key_from_vision=self.information)
        else:
            self.add('right','restore_install_holding',self.initial_ee['right'])
        self.phase='recovering'
        self.event(t,'recovery_start')

    def tick(self,t):
        idle=lambda a:self.active[a] is None and not self.queues[a]
        if self.phase=='new':
            if self.route=='putdown':
                self.phase='placing'
                self.add('right','rack_pre',self.candidate['goal'])
                seat=self.rack_module.copy();seat[2,3]+=.001
                self.add('right','rack_lower',module_to_ee(seat),cartesian=True)
                self.add('right','rack_release',grip=.045)
                withdraw=self.rack_module.copy();withdraw[2,3]+=.06
                self.add('right','rack_withdraw',module_to_ee(withdraw),cartesian=True)
            else:
                self.phase='observing'
                self.add(self.candidate['arm'],'observe_candidate',self.candidate['goal'])
            self.event(t,'observation_start',route=self.route,candidate=self.candidate)
        if self.status['right'] in ['rack_release_done','rack_withdraw','rack_withdraw_done']:self.held=False
        if self.route=='putdown' and self.phase=='placing' and idle('right'):
            self.phase='rack_observing'
            center=np.array(self.config['rack_center'])+[0,0,-.015]
            # Empty wrist below the edge-supported plate; public rack pose only.
            side=module_to_ee(self.rack_module);side[:3,3]+=[.09,-.12,.06]
            self.add('right','empty_withdraw_side',side)
            camera=look_at(np.array([.50,-.16,.83]),center)@transform(rotation=Rotation.from_euler('x',180,degrees=True).as_matrix())
            self.add('right','observe_rack',camera@np.linalg.inv(T_EE_CAMERA))
        if self.route=='putdown' and self.phase=='rack_observing' and idle('right'):
            if self.rack_view_done_since is None:self.rack_view_done_since=t
            if self.information is not None or t-self.rack_view_done_since>=.4:
                self.phase='regrasp'
                pre=self.rack_module.copy();pre[2,3]+=.06
                side=module_to_ee(pre);side[:3,3]+=[.09,-.12,0]
                self.add('right','regrasp_side',side)
                self.add('right','regrasp_pre',module_to_ee(pre))
                self.add('right','regrasp_down',module_to_ee(self.rack_module),cartesian=True)
                self.add('right','regrasp_close',grip=0.)
                self.add('right','regrasp_lift',module_to_ee(pre),cartesian=True)
                self.event(t,'regrasp_start')
        if self.phase=='regrasp' and idle('right'):
            self.held=True;self.phase='observed';self.event(t,'regrasp_command_complete')
        if self.phase=='observing' and idle(self.candidate['arm']):
            if self.done_since is None:self.done_since=t
            elif t-self.done_since>=.4:self.phase='observed'
        if self.phase=='observed' and not self.recovery_started:self.recover(t)
        if self.phase=='recovering' and all(idle(a) for a in ['left','right']):
            self.phase='done';self.event(t,'recovery_complete')
        for arm in ['left','right']:self.advance(arm,t)
        self.kin.compensate()
