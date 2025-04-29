# camera_intrinsics.py
import numpy as np
class CameraIntrinsics:
    def __init__(self, fx, fy, cx, cy):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy

    def back_project(self, u, v, d):
        print(u.shape)
        print(v.shape)
        print(d.shape)
        X = (u - self.cx) * d / self.fx
        Y = (v - self.cy) * d / self.fy
        Z = d
        return np.array([X, Y, Z], dtype=np.float32)
    
    def get_camera_matrix(self):
        return np.array([[self.fx, 0, self.cx],
                         [0, self.fy, self.cy],
                         [0, 0, 1]], dtype=np.float32)