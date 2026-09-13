"""Check moving-key vision and the policy/GT boundary, without simulator access."""
import ast
import unittest
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from envs.occluded_socket.geometry import transform
from envs.occluded_socket.observation import CameraFrame
from envs.held_module_key.perception import KeyPerception


class Contracts(unittest.TestCase):
    def test_full_pose_and_motion_transport(self):
        points=np.array([[-.010,-.005,-.0121],[.010,-.005,-.0121],[.010,.005,-.0121],[-.010,.005,-.0121]])
        perception=KeyPerception(points,dict(min_pixels=5,geometry_error=.0025,temporal_translation=.004,temporal_angle_degrees=8))
        K=np.array([[500.,0,320],[0,500.,240],[0,0,1]])
        relative=transform([0,0,.18],Rotation.from_euler('xyz',[12,-20,35],degrees=True).as_matrix())
        detections=[]
        for i,x in enumerate([0,.025]):
            ee=transform([x,0,0],Rotation.from_euler('y',i*.15).as_matrix())
            T=ee@relative
            camera_points=points@T[:3,:3].T+T[:3,3]
            uv=camera_points@K.T;uv=uv[:,:2]/uv[:,2:]
            rgb=np.zeros((480,640,3),dtype=np.uint8);depth=np.zeros((480,640),dtype=np.float32)
            for pixel,p,color in zip(uv,camera_points,[(255,0,0),(0,255,0),(0,0,255),(255,255,0)]):
                pixel=tuple(np.round(pixel).astype(int))
                cv2.circle(rgb,pixel,4,color,-1);cv2.circle(depth,pixel,4,float(p[2]),-1)
            frame=CameraFrame('head',i*.1,rgb,depth,K,np.eye(4))
            detections.append(perception.detect(frame,ee))
        self.assertFalse(detections[0]['valid'])
        self.assertTrue(detections[1]['valid'])
        estimate=detections[1]['T_ee_key']
        self.assertLess(np.linalg.norm(estimate[:3,3]-relative[:3,3]),.002)
        self.assertLess(Rotation.from_matrix(estimate[:3,:3]@relative[:3,:3].T).magnitude(),np.deg2rad(3))

    def test_policy_has_no_privileged_state_access(self):
        root=Path(__file__).resolve().parents[2]
        for name in ['controller.py','perception.py']:
            tree=ast.parse((root/'envs/held_module_key'/name).read_text())
            for node in ast.walk(tree):
                if isinstance(node,ast.Attribute):
                    self.assertNotIn(node.attr,{'scene','module','T_module_key','seed','module_body','get_pose','set_pose','set_qpos'})


if __name__=='__main__':unittest.main()
