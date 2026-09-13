"""H/S finite state machines. Inputs are RGB-D detections and robot-only APIs.

No seed, LOW/HIGH label, environment actor, scene, GT collision world or evaluator.
"""
from collections import deque
import time
import cv2
import numpy as np
from .geometry import transform, look_at, T_EE_CAMERA, module_to_ee


R_PUSH = np.array([[0., -1, 0], [np.cos(.26), 0, np.sin(.26)], [-np.sin(.26), 0, np.cos(.26)]])


def detect_drawer_handle(frame):
    hsv=cv2.cvtColor(frame.rgb,cv2.COLOR_RGB2HSV)
    mask=cv2.inRange(hsv,np.array([140,110,65]),np.array([171,255,255]))
    n,labels,stats,centers=cv2.connectedComponentsWithStats(mask)
    valid=[i for i in range(1,n) if stats[i,cv2.CC_STAT_AREA]>8]
    if not valid:
        return None
    i=max(valid,key=lambda x:stats[x,cv2.CC_STAT_AREA])
    z=np.median(frame.depth[labels==i])
    if not .02<z<2:
        return None
    xyz=np.linalg.inv(frame.intrinsic)@[*centers[i],1]*z
    return frame.T_world_camera[:3,:3]@xyz+frame.T_world_camera[:3,3]


class Controller:
    def __init__(self, policy, kinematics, public_config, log, variant='baseline'):
        self.policy, self.kin, self.config, self.log = policy, kinematics, public_config, log
        self.variant=variant
        self.queues={a:deque() for a in ['left','right']}
        self.active={a:None for a in self.queues}
        self.status={'left':'ready','right':'holding'}
        self.grip={'left':0.,'right':0.}
        self.information=None
        self.t_information_ready=None
        self.drawer_handle=None
        self.drawer_started=False
        self.install_started=False
        self.right_released=False
        self.failure=None
        self.planning_wall=0.
        self.views={'left': [[-.08,-.12,1.0],[0,-.22,1.04],[.05,-.24,.94]],
                    'right': [[.35,-.28,1.07],[.35,-.28,1.07],[.37,-.12,1.06]]}
        self.view_index={'left':0,'right':0}
        self.observe_since=None
        self.started=False
        self.observation_enabled=True
        self.partial_phase=None
        self.handoff_phase=None

    def event(self, t, event, **data):
        self.log(dict(timestamp=t,event=event,**data))

    def add(self, arm, name, goal=None, grip=None, cartesian=False):
        self.queues[arm].append(dict(name=name,goal=goal,grip=grip,cartesian=cartesian))

    def observing(self, arm, t):
        if self.view_index[arm]>=len(self.views[arm]):
            self.failure='perception_candidates_exhausted'
            self.event(t,'failure',reason=self.failure)
            return
        i=self.view_index[arm]
        self.view_index[arm]+=1
        camera=look_at(np.array(self.views[arm][i]),self.config['socket_center'])
        if arm=='right':
            # Fixed image offset exposes the free half of the wrist image while held.
            aim=np.array(self.config['socket_center'])-camera[:3,1]*(.10 if i==0 else .16)
            camera=look_at(np.array(self.views[arm][i]),aim)
        self.add(arm,f'observe_{i}',camera@np.linalg.inv(T_EE_CAMERA))
        self.observe_since=None

    def drawer(self, t, arm='left', partial=False):
        if self.drawer_handle is None:
            return
        self.drawer_started=True
        closed=np.array(self.config['drawer_center'])+[0,-.172,.045]
        # Front surface is visible, known handle geometry gives fixed contact offset.
        observed=self.drawer_handle.copy()
        # Known one-DOF guide: only y is unknown. Project visible surface points
        # onto the calibrated drawer axis instead of following partial-mask x/z bias.
        observed[0]=self.config['drawer_center'][0]
        observed[2]=self.config['drawer_center'][2]+.045
        if closed[1]-observed[1] < .009:
            self.status[arm]='drawer_already_closed'
            self.event(t,'drawer_no_work',arm=arm,observed_handle=observed)
            return
        pre=observed-R_PUSH[:,0]*.18
        contact=observed-R_PUSH[:,0]*.147
        end=closed-R_PUSH[:,0]*.14
        if partial:
            end=(contact+end)/2
        # Rotate/translate above the drawer before descending behind its front.
        hover=pre.copy();hover[2]=1.08
        self.add(arm,'drawer_hover',transform(hover,R_PUSH))
        self.add(arm,'drawer_approach',transform(pre,R_PUSH),cartesian=True)
        self.add(arm,'drawer_contact',transform(contact,R_PUSH),cartesian=True)
        self.add(arm,'drawer_push',transform(end,R_PUSH),cartesian=True)
        if not partial:
            self.add(arm,'drawer_retreat',transform(end+[0,-.05,.08],R_PUSH),cartesian=True)
        self.event(t,'drawer_plan',arm=arm,observed_handle=observed,ee_goal=end,partial=partial)

    def install(self,t):
        self.install_started=True
        T=self.information.copy()
        T[2,3]+=self.config['module_half_size'][2]
        pre=T.copy();pre[2,3]+=.15
        seat=T.copy();seat[2,3]+=.002
        retreat=module_to_ee(seat);retreat[2,3]+=.13
        self.add('right','install_preapproach',module_to_ee(pre))
        self.add('right','install_insert',module_to_ee(seat),cartesian=True)
        self.add('right','install_release',grip=.045)
        self.add('right','install_retreat',retreat,cartesian=True)
        self.event(t,'installation_target',T_world_socket=self.information,T_world_ee_goal=module_to_ee(seat))

    def observe(self,t,frames,detections):
        # Both wrists and the permitted head are checked during every task role.
        handle=detect_drawer_handle(frames['head'])
        if handle is not None:
            self.drawer_handle=handle
            self.kin.update_drawer_obstacle(handle)
        for d in detections:
            if d.valid and self.information is None:
                self.information=d.T_world_socket.copy()
                self.t_information_ready=t
                self.event(t,'information_ready',camera=d.camera,T_world_socket=self.information,
                           incidental=not any('observe' in s for s in self.status.values()))
                arm='left' if self.policy=='H' else 'right'
                if self.active[arm] is not None and self.active[arm]['name'].startswith('observe'):
                    self.event(t,'action_cancel',arm=arm,name=self.active[arm]['name'],reason='vision_ready')
                    self.active[arm]=None
                    self.queues[arm].clear()

    def tick(self,t):
        if not self.started:
            self.started=True
            if self.policy=='S':
                self.drawer(t)
            if self.policy=='H' and self.variant=='partial_helper' and self.drawer_handle is not None:
                self.drawer(t,partial=True)
                self.partial_phase='pushing'
            elif self.information is None and self.observation_enabled:
                self.observing('left' if self.policy=='H' else 'right',t)
        if self.partial_phase=='pushing' and self.active['left'] is None and not self.queues['left']:
            self.partial_phase='observing'
            self.drawer_started=False
            self.event(t,'partial_drawer_complete')
            if self.information is None:
                self.observing('left',t)
        if self.policy=='S' and not self.drawer_started:
            self.drawer(t)
        if self.information is not None and not self.install_started:
            self.install(t)
            if self.policy=='H' and self.partial_phase!='pushing':
                self.drawer(t)
        if self.policy=='H' and self.install_started and not self.drawer_started and self.partial_phase!='pushing':
            self.drawer(t)
        if self.variant=='right_handoff' and self.policy=='H' and self.install_started:
            if self.handoff_phase is None and self.status['right']=='install_retreat_done':
                self.handoff_phase='checked'
                closed_y=self.config['drawer_center'][1]-.172
                if self.drawer_handle is not None and closed_y-self.drawer_handle[1]>.009:
                    self.event(t,'handoff_attempt',observed_handle=self.drawer_handle)
                    if self.active['left'] is not None:
                        self.event(t,'action_cancel',arm='left',name=self.active['left']['name'],reason='right_handoff')
                    self.active['left']=None;self.queues['left'].clear()
                    clearance=self.kin.fk('left');clearance[2,3]=1.20
                    self.add('left','handoff_clearance',clearance,cartesian=True)
                    self.handoff_phase='clearing'
                else:
                    self.event(t,'handoff_unnecessary',reason='drawer_visually_closed_or_unobserved')
            if self.handoff_phase=='clearing' and self.status['left']=='handoff_clearance_done':
                self.handoff_phase='executing'
                self.drawer(t,arm='right')
        observer='left' if self.policy=='H' else 'right'
        if self.observation_enabled and self.information is None and self.active[observer] is None and not self.queues[observer] and not self.failure:
            if self.observe_since is None:
                self.observe_since=t
            elif t-self.observe_since>=.4:
                self.observing(observer,t)
        for arm in ['left','right']:
            self.advance(arm,t)
        self.kin.compensate()

    def advance(self,arm,t):
        current=self.active[arm]
        if current is None and self.queues[arm]:
            current=self.queues[arm].popleft()
            planning_started=time.perf_counter()
            try:
                if current['goal'] is not None:
                    planner=self.kin.cartesian_path if current['cartesian'] else self.kin.joint_path
                    current['position'],current['velocity'],wall=planner(arm,current['goal'])
                    current['length']=len(current['position'])
                else:
                    current['length']=int(.5/self.config['dt'])
                    current['grip_start']=self.grip[arm]
                current['index']=0
                self.active[arm]=current
                self.status[arm]=current['name']
                if current['goal'] is not None:
                    other='right' if arm=='left' else 'left'
                    other_action=self.active[other]
                    other_path=None if other_action is None or 'position' not in other_action else other_action['position'][other_action['index']:]
                    contact=current['name'].startswith(('drawer_contact','drawer_push','drawer_retreat'))
                    moving_drawer=current['name']=='drawer_push'
                    other_contact=other_action is not None and other_action['name'].startswith(('drawer_contact','drawer_push','drawer_retreat'))
                    other_moving=other_action is not None and other_action['name']=='drawer_push'
                    check_options=dict(contact=contact,moving_drawer=moving_drawer,other_contact=other_contact,other_moving_drawer=other_moving)
                    hits=self.kin.check_path(arm,current['position'],other_path,**check_options)
                    if hits and not current['cartesian']:
                        self.event(t,'avoidance_replan',arm=arm,name=current['name'],collisions=hits)
                        current['position'],current['velocity'],wall=self.kin.clearance_path(arm,current['goal'])
                        current['length']=len(current['position'])
                        hits=self.kin.check_path(arm,current['position'],other_path,**check_options)
                    self.event(t,'path_collision_check',arm=arm,name=current['name'],collisions=hits)
                    self.event(t,'robot_tracking',arm=arm,actual_fk=self.kin.fk(arm),goal=current['goal'])
                    if hits:
                        self.active[arm]=None
                        raise RuntimeError(f'known_geometry_collision: {hits}')
                self.event(t,'action_start',arm=arm,name=current['name'],ticks=current['length'])
                if current['goal'] is not None:
                    self.planning_wall+=time.perf_counter()-planning_started
            except RuntimeError as e:
                self.active[arm]=None
                self.planning_wall+=time.perf_counter()-planning_started
                self.event(t,'planning_failure',arm=arm,name=current['name'],error=str(e))
                self.status[arm]='planning_failed'
                self.queues[arm].clear()
                self.failure=None if current['name'].startswith('observe') else 'planning_failure'
                if current['name'].startswith('observe'):
                    # A rejected path has no new viewpoint to stabilize/render.
                    self.observe_since=t-.4
                return
        if current is not None:
            i=current['index']
            if current['goal'] is not None:
                self.kin.drive(arm,current['position'][i],current['velocity'][i])
            else:
                self.grip[arm]=current['grip_start']+(current['grip']-current['grip_start'])*(i+1)/current['length']
            current['index']+=1
            if current['index']==current['length']:
                self.event(t+self.config['dt'],'action_end',arm=arm,name=current['name'])
                if current['name']=='install_release':
                    self.right_released=True
                self.active[arm]=None
                self.status[arm]=current['name']+'_done'
        self.kin.gripper(arm,self.grip[arm])
