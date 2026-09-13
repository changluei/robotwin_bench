"""Robot-only kinematics and bounded trajectories; never receives task actors."""
import time
import numpy as np
import sapien
import mplib
from mplib.collision_detection import fcl
from scipy.spatial.transform import Rotation, Slerp
from scipy.interpolate import CubicSpline


class RobotKinematics:
    def __init__(self, robot, config):
        self.robot = robot
        self.entity = robot.left_entity
        self.model = self.entity.create_pinocchio_model()
        self.root = self.entity.get_root_pose().to_transformation_matrix()
        self.config = config
        joints = self.entity.get_active_joints()
        links = self.entity.get_links()
        self.ids = {a: [joints.index(j) for j in getattr(robot, a + '_arm_joints')] for a in ['left', 'right']}
        self.link_ids = {a: next(i for i, link in enumerate(links) if link.name == ('fl_link6' if a == 'left' else 'fr_link6')) for a in self.ids}
        self.limits = self.entity.get_qlimits()
        self.collision_planner=mplib.Planner(robot.left_urdf_path,move_group='fl_link6',srdf=robot.left_srdf_path,
                    user_link_names=[l.name for l in links],user_joint_names=[j.name for j in joints])
        self.collision_planner.set_base_pose(mplib.Pose(self.root))
        # Public fixed obstacles ONLY. No hidden socket/marker geometry in planner.
        center=np.array(config['socket_center'])
        boxes=[('table',[0,0,.715],[1.2,.72,.05]),
               ('housing_floor',center+[0,0,-.02],[.24,.2,.02]),
               ('housing_right',center+[.112,0,.055],[.016,.2,.13]),
               ('housing_back',center+[0,.12,.055],[.24,.016,.13])]
        for name,p,size in boxes:
            self.collision_planner.planning_world.add_object(name,fcl.CollisionObject(fcl.Box(size),mplib.Pose(p,[1,0,0,0])))

    def update_drawer_obstacle(self, handle):
        # Only image-estimated moving geometry. Random socket remains excluded.
        c=np.array(self.config['drawer_center'],dtype=float)
        c[1]=float(handle[1])+.16
        self.drawer_estimate=c.copy()
        self._set_drawer_center(c)

    def _set_drawer_center(self,c):
        boxes=[('drawer_estimated_floor',c,[.208,.276,.028]),
               ('drawer_estimated_front',c+[0,-.132,.034],[.226,.032,.102])]
        for name,p,size in boxes:
            self.collision_planner.planning_world.remove_object(name)
            self.collision_planner.planning_world.add_object(name,fcl.CollisionObject(fcl.Box(size),mplib.Pose(p,[1,0,0,0])))

    def check_path(self, arm, path, other_path=None, contact=False, moving_drawer=False,
                   other_contact=False,other_moving_drawer=False):
        """Sample planned simultaneous joint states against robot + known fixtures."""
        q=self.entity.get_qpos().copy()
        other='right' if arm=='left' else 'left'
        pushing_arm=arm if moving_drawer else other
        predict_drawer=moving_drawer or other_moving_drawer
        initial_y=self.fk(pushing_arm,q)[1,3]
        drawer=getattr(self,'drawer_estimate',None)
        for index in range(0,len(path),10):
            q[self.ids[arm]]=path[index]
            if other_path is not None and len(other_path):
                q[self.ids[other]]=other_path[min(index,len(other_path)-1)]
            if predict_drawer and drawer is not None:
                predicted=drawer.copy()
                predicted[1]+=max(0,self.fk(pushing_arm,q)[1,3]-initial_y)
                self._set_drawer_center(predicted)
            hits=self.collision_planner.check_for_self_collision(q)+self.collision_planner.check_for_env_collision(q)
            # The URDF also contains unused rear arms and wheels; their adjacent
            # fixed/home contacts are irrelevant to the two active front arms.
            hits=[h for h in hits if any(n.startswith(('fl_link','fr_link')) or n in ['left_camera','right_camera']
                                          for n in [h.link_name1,h.link_name2])]
            if contact or other_contact:
                fingers=set()
                for a,allowed in [(arm,contact),(other,other_contact)]:
                    if allowed:
                        fingers.update({'fl_link7','fl_link8'} if a=='left' else {'fr_link7','fr_link8'})
                hits=[h for h in hits if not ((h.link_name1 in fingers and h.link_name2.startswith('drawer_estimated'))
                                          or (h.link_name2 in fingers and h.link_name1.startswith('drawer_estimated')))]
            if hits:
                if predict_drawer and drawer is not None:
                    self._set_drawer_center(drawer)
                return [dict(a=h.link_name1,b=h.link_name2) for h in hits]
        if predict_drawer and drawer is not None:
            self._set_drawer_center(drawer)
        return []

    def clearance_path(self,arm,goal):
        start_q=self.entity.get_qpos().copy()
        start=self.fk(arm,start_q)
        withdraw=start.copy()
        if start[2,0]>-.7:
            withdraw[:3,3]-=start[:3,0]*.08
        up=start.copy();up[2,3]=max(1.15,start[2,3])
        up[:2,3]=withdraw[:2,3]
        across=up.copy();across[:2,3]=goal[:2,3]
        paths=[];velocities=[];wall=0.
        for waypoint in [withdraw,up,across,goal]:
            if np.linalg.norm(self.fk(arm,start_q)-waypoint)<1e-4:
                continue
            path,vel,seconds=self.joint_path(arm,waypoint,start_q)
            paths.append(path);velocities.append(vel);wall+=seconds
            start_q[self.ids[arm]]=path[-1]
        return np.concatenate(paths),np.concatenate(velocities),wall

    def fk(self, arm, qpos=None):
        qpos = self.entity.get_qpos() if qpos is None else qpos
        self.model.compute_forward_kinematics(qpos)
        return self.root @ self.model.get_link_pose(self.link_ids[arm]).to_transformation_matrix()

    def ik(self, arm, goal, qpos=None):
        start = self.entity.get_qpos() if qpos is None else qpos.copy()
        mask = np.zeros(len(start), dtype=np.int32)
        mask[self.ids[arm]] = 1
        target = sapien.Pose(np.linalg.inv(self.root) @ goal)
        for attempt in range(4):
            initial = start.copy()
            if attempt:
                # Fixed numerical seeds, independent of the instance random seed.
                initial[self.ids[arm]] += np.array([0, .25, -.4, 0, .2, 0]) * attempt
            q, success, error = self.model.compute_inverse_kinematics(self.link_ids[arm], target,
                        initial_qpos=initial, active_qmask=mask, eps=1e-5, max_iterations=250, damp=1e-5)
            # Choose equivalent angles nearest start and enforce actual joint limits.
            for i in self.ids[arm]:
                q[i] += 2*np.pi * np.round((start[i] - q[i])/(2*np.pi))
            if success and np.all(q >= self.limits[:, 0] - 1e-5) and np.all(q <= self.limits[:, 1] + 1e-5):
                return q
        raise RuntimeError(f'IK failed for {arm}: position={goal[:3,3].tolist()}, error={error.tolist()}')

    def joint_path(self, arm, goal, qpos=None):
        tic = time.perf_counter()
        start = self.entity.get_qpos() if qpos is None else qpos.copy()
        seed=start.copy()
        if hasattr(self,'reference_qpos'):
            seed[self.ids[arm]]=self.reference_qpos[self.ids[arm]]
        end = self.ik(arm, goal, seed)
        for i in self.ids[arm]:
            end[i]+=2*np.pi*np.round((start[i]-end[i])/(2*np.pi))
        ids = self.ids[arm]
        delta = end[ids] - start[ids]
        # Quintic blend has max first/second derivatives 1.875 / 5.774.
        duration = max(1.875 * np.max(abs(delta)) / self.config['joint_velocity'],
                       np.sqrt(5.774 * np.max(abs(delta)) / self.config['joint_acceleration']), .08)
        n = int(np.ceil(duration / self.config['dt']))
        t = np.linspace(0, 1, n+1)
        blend = 10*t**3-15*t**4+6*t**5
        vel = (30*t**2-60*t**3+30*t**4) / (n*self.config['dt'])
        return start[ids] + blend[:,None]*delta, vel[:,None]*delta, time.perf_counter()-tic

    def drive(self, arm, position, velocity=None):
        # No step, teleport, actor state or camera operation in this method.
        joints = getattr(self.robot, arm + '_arm_joints')
        velocity = np.zeros(len(joints)) if velocity is None else velocity
        for j, p, v in zip(joints, position, velocity):
            j.set_drive_target(float(p))
            j.set_drive_velocity_target(float(v))

    def cartesian_path(self, arm, goal):
        tic = time.perf_counter()
        q = self.entity.get_qpos().copy()
        start = self.fk(arm, q)
        distance = np.linalg.norm(goal[:3,3]-start[:3,3])
        angle = Rotation.from_matrix(start[:3,:3].T@goal[:3,:3]).magnitude()
        duration = max(1.875*distance/self.config['cartesian_velocity'],
                       np.sqrt(5.774*distance/self.config['cartesian_acceleration']), 1.875*angle/.7, .2)
        n = max(2,int(np.ceil(duration/.04)))
        rotations = Slerp([0,1],Rotation.from_matrix(np.stack([start[:3,:3],goal[:3,:3]])))
        path = []
        for u in np.linspace(0,1,n+1):
            blend=10*u**3-15*u**4+6*u**5
            target=np.eye(4)
            target[:3,:3]=rotations(blend).as_matrix()
            target[:3,3]=start[:3,3]+blend*(goal[:3,3]-start[:3,3])
            q=self.ik(arm,target,q)
            path.append(q[self.ids[arm]].copy())
        path=np.array(path)
        sample_dt=duration/n
        velocity=np.gradient(path,sample_dt,axis=0)
        acceleration=np.gradient(velocity,sample_dt,axis=0)
        scale=max(np.max(abs(velocity))/self.config['joint_velocity'],
                  np.sqrt(np.max(abs(acceleration))/self.config['joint_acceleration']),1.)
        duration*=scale
        # C2 interpolation avoids acceleration spikes at the 40 ms IK knots.
        # Check the actual 4 ms command grid, then stretch physical duration if
        # needed. Both policies share these exact velocity/acceleration bounds.
        spline=CubicSpline(np.linspace(0,1,n+1),path,axis=0,
                           bc_type=((1,np.zeros(6)),(1,np.zeros(6))))
        for _ in range(4):
            steps=int(np.ceil(duration/self.config['dt']))
            duration=steps*self.config['dt']
            phase=np.linspace(0,1,steps+1)
            vel=spline(phase,1)/duration
            acc=spline(phase,2)/duration**2
            stretch=max(np.max(abs(vel))/self.config['joint_velocity'],
                        np.sqrt(np.max(abs(acc))/self.config['joint_acceleration']),1.)
            if stretch<=1+1e-8:
                break
            duration*=stretch*1.001
        out=spline(phase)
        return out,vel,time.perf_counter()-tic

    def compensate(self):
        self.entity.set_qf(self.entity.compute_passive_force(gravity=True, coriolis_and_centrifugal=True))

    def gripper(self, arm, opening):
        # Smooth opening is handled by the queue; use actual position drives.
        for joint, multiplier, offset in getattr(self.robot, arm + '_gripper'):
            joint.set_drive_target(float(opening*multiplier+offset))
            joint.set_drive_velocity_target(0)
