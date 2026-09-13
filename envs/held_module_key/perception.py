"""Full RGB-D key pose from paint geometry, transported by robot FK only."""
from itertools import product
import cv2
import numpy as np
from scipy.spatial.transform import Rotation


class KeyPerception:
    def __init__(self,points,thresholds):
        self.points=np.array(points)
        self.thresholds=thresholds
        self.previous={}

    def detect(self,frame,T_world_ee,held=True):
        hsv=cv2.cvtColor(frame.rgb,cv2.COLOR_RGB2HSV)
        h,s,v=cv2.split(hsv)
        masks=[(h<9)|(h>172),(h>42)&(h<82),(h>102)&(h<132),(h>20)&(h<38)]
        alternatives=[]
        Kinv=np.linalg.inv(frame.intrinsic)
        for mask in masks:
            n,labels,stats,centers=cv2.connectedComponentsWithStats((mask&(s>130)&(v>55)).astype('uint8'))
            candidates=[]
            for i in range(1,n):
                count=stats[i,cv2.CC_STAT_AREA]
                if count<self.thresholds['min_pixels']:continue
                z=float(np.median(frame.depth[labels==i]))
                width=max(stats[i,cv2.CC_STAT_WIDTH],stats[i,cv2.CC_STAT_HEIGHT])*z/frame.intrinsic[0,0]
                if .015<z<2 and .002<width<.013:
                    candidates.append((centers[i],count,Kinv@[*centers[i],1]*z))
            alternatives.append(sorted(candidates,key=lambda a:-a[1])[:5])
        result=dict(timestamp=frame.timestamp,camera=frame.name,valid=False,reason='incomplete_markers',
                    counts=[a[0][1] if a else 0 for a in alternatives])
        if not all(alternatives):
            self.previous.pop(frame.name,None)
            return result
        def fit(combination):
            measured=np.array([x[2] for x in combination])
            source=self.points-self.points.mean(0)
            U,_,Vt=np.linalg.svd(source.T@(measured-measured.mean(0)))
            R=Vt.T@np.diag([1,1,np.linalg.det(Vt.T@U.T)])@U.T
            p=measured.mean(0)-R@self.points.mean(0)
            prediction=self.points@R.T+p
            error=float(np.max(np.linalg.norm(prediction-measured,axis=1)))
            T=np.eye(4);T[:3,:3]=R;T[:3,3]=p
            return error,T,prediction
        chosen=min(product(*alternatives),key=lambda a:fit(a)[0])
        error,T_camera_key,prediction=fit(chosen)
        projection=prediction@frame.intrinsic.T
        pixels=np.array([x[0] for x in chosen])
        reproj=float(np.sqrt(np.mean(np.sum((projection[:,:2]/projection[:,2:]-pixels)**2,axis=1))))
        T_world_key=frame.T_world_camera@T_camera_key
        T_ee_key=np.linalg.inv(T_world_ee)@T_world_key
        result.update(geometry_error=error,reprojection_error=reproj,pixels=pixels,
                      counts=[x[1] for x in chosen],T_world_key=T_world_key,T_ee_key=T_ee_key)
        if error>self.thresholds['geometry_error'] or reproj>3:
            result['reason']='geometry_or_reprojection';self.previous.pop(frame.name,None)
            return result
        relative=T_ee_key if held else T_world_key
        previous=self.previous.get(frame.name)
        self.previous[frame.name]=(relative,held)
        consistent=previous is not None and previous[1]==held and np.linalg.norm(relative[:3,3]-previous[0][:3,3])<self.thresholds['temporal_translation'] and Rotation.from_matrix(relative[:3,:3]@previous[0][:3,:3].T).magnitude()<np.deg2rad(self.thresholds['temporal_angle_degrees'])
        result.update(valid=bool(consistent),reason='valid' if consistent else 'await_consistency')
        return result
