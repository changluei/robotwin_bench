"""CPU contract tests; actual simulation evidence is checked separately."""
import ast
import sys
import unittest
from types import SimpleNamespace
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from envs.occluded_socket.geometry import T_SAPIEN_CV,T_EE_MODULE,R_DOWN,transform,module_to_ee
from envs.occluded_socket.observation import CameraFrame
from envs.occluded_socket.perception import Perception
from envs.occluded_socket.controller import Controller
from envs.occluded_socket.motion import RobotKinematics
from envs.occluded_socket.evaluation import Evaluator


class Contracts(unittest.TestCase):
    def test_camera_axes(self):
        np.testing.assert_allclose(T_SAPIEN_CV[:3,:3]@[0,0,1],[1,0,0])
        np.testing.assert_allclose(T_SAPIEN_CV[:3,:3]@[1,0,0],[0,-1,0])
        np.testing.assert_allclose(T_SAPIEN_CV[:3,:3]@[0,1,0],[0,0,-1])
        self.assertAlmostEqual(np.linalg.det(T_SAPIEN_CV[:3,:3]),1)

    def test_grasp_transform_and_target_dependence(self):
        a=transform([.18,-.04,.78]);b=transform([.191,-.047,.78])
        np.testing.assert_allclose(module_to_ee(a)@T_EE_MODULE,a,atol=1e-12)
        np.testing.assert_allclose((module_to_ee(b)-module_to_ee(a))[:3,3],[.011,-.007,0],atol=1e-12)

    def test_blank_image_cannot_produce_target(self):
        thresholds=dict(min_pixels=10,min_colors=4,max_geometry_error=.006,max_reprojection_error=3,
                        max_temporal_translation=.006,consistent_frames=2)
        p=Perception([[-.063,-.056,.014],[.063,-.056,.014],[.063,.056,.014],[-.063,.056,.014]],thresholds)
        f=CameraFrame('right',0,np.zeros((120,160,3),np.uint8),np.ones((120,160)),np.eye(3),np.eye(4))
        for _ in range(4):
            d=p.detect(f)
            self.assertFalse(d.valid);self.assertIsNone(d.T_world_socket)

    def test_policy_gt_boundary(self):
        forbidden={'get_pose','get_functional_point','grasp_actor','place_actor','get_segmentation','get_world_pcd',
                   'set_pose','set_qpos','step','snapshot','socket_xyz','seed','load_condition','evaluator'}
        for name in ['controller.py','perception.py']:
            tree=ast.parse((ROOT/'envs/occluded_socket'/name).read_text())
            attributes={n.attr for n in ast.walk(tree) if isinstance(n,ast.Attribute)}
            self.assertFalse(attributes&forbidden,(name,attributes&forbidden))
        source=(ROOT/'envs/occluded_socket/motion.py').read_text()
        self.assertNotIn('SapienPlanningWorld',source)
        self.assertNotIn("'hidden_socket'",source)

    def test_one_post_checkpoint_step_site(self):
        tree=ast.parse((ROOT/'envs/occluded_socket/runtime.py').read_text())
        run=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='run_episode')
        calls=[n for n in ast.walk(run) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='step_physics']
        self.assertEqual(len(calls),1)

    def test_rejected_path_never_executes(self):
        class RobotOnly:
            def joint_path(self,*args):
                return np.zeros((20,6)),np.zeros((20,6)),0.
            def check_path(self,*args,**kwargs):
                return [dict(a='fl_link7',b='fixture')]
            def clearance_path(self,*args):
                raise RuntimeError('unreachable clearance')
            def gripper(self,*args):
                pass
            def drive(self,*args):
                raise AssertionError('rejected trajectory executed')
        c=Controller('H',RobotOnly(),{'dt':.004},lambda event:None)
        c.add('left','observe_0',np.eye(4))
        c.advance('left',0)
        for i in range(30):
            c.advance('left',i*.004)
        self.assertIsNone(c.active['left'])
        self.assertEqual(c.status['left'],'planning_failed')

    def test_cartesian_commands_respect_joint_limits_at_physics_rate(self):
        # Independent smooth nonlinear IK fixture exposes acceleration spikes
        # from coarse-to-fine piecewise-linear resampling, without a GPU scene.
        k=RobotKinematics.__new__(RobotKinematics)
        k.config=dict(dt=.004,cartesian_velocity=.12,cartesian_acceleration=.3,
                      joint_velocity=.8,joint_acceleration=1.6)
        k.entity=type('JointState',(),{'get_qpos':lambda self:np.zeros(6)})()
        k.ids={'left':list(range(6))}
        k.fk=lambda arm,q=None:np.eye(4)
        k.ik=lambda arm,T,q:np.array([5*T[0,3],2*np.sin(3*T[0,3]),0,0,0,0])
        goal=np.eye(4);goal[0,3]=.2
        path,velocity,_=k.cartesian_path('left',goal)
        self.assertLessEqual(np.max(abs(velocity)),.8+1e-5)
        self.assertLessEqual(np.max(abs(np.gradient(velocity,.004,axis=0))),1.6+1e-3)
        np.testing.assert_allclose(path[-1,:2],[1,2*np.sin(.6)],atol=1e-8)

    def test_success_is_revoked_if_module_later_leaves_socket(self):
        T=np.eye(4);T[2,3]=.008
        pose=SimpleNamespace(to_transformation_matrix=lambda:T)
        joint1,joint2=object(),object()
        robot=SimpleNamespace(right_entity=SimpleNamespace(get_active_joints=lambda:[joint1,joint2],
                              get_qpos=lambda:np.array([.045,.045])),right_gripper=[(joint1,1,0),(joint2,1,0)])
        evaluation=dict(position_xy=.015,position_z=.008,angle_degrees=12,stable_seconds=.5,
                        linear_speed=.015,angular_speed=.1,drawer_closed=.006,drawer_speed=.01)
        env=SimpleNamespace(config={'evaluation':evaluation,'module_half_size':[.038,.032,.008]},
            module=SimpleNamespace(get_pose=lambda:pose),socket=SimpleNamespace(get_pose=lambda:SimpleNamespace(to_transformation_matrix=lambda:np.eye(4))),
            module_body=SimpleNamespace(linear_velocity=np.zeros(3),angular_velocity=np.zeros(3)),
            robot=robot,include_drawer=True,drawer=SimpleNamespace(get_qpos=lambda:[0.],get_qvel=lambda:[0.]))
        evaluator=Evaluator(env)
        evaluator.sample(0,True);evaluator.sample(.5,True)
        self.assertTrue(evaluator.current['all'])
        T[2,3]=.15
        evaluator.sample(.504,True)
        self.assertFalse(evaluator.current['all'])
        self.assertIsNone(evaluator.times['right']);self.assertIsNone(evaluator.times['all'])


if __name__=='__main__':
    unittest.main()
