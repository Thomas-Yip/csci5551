# aruco_marker.py
import cv2
import numpy as np
from cv_bridge import CvBridge
from camera_intrinsics import CameraIntrinsics
class ArucoMarker:
    def __init__(self, camera_intrinsics):
        # Initialize the ArUco dictionary and parameters
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        # Initialize the state vector and covariance matrix
        self.Xr = np.zeros((3, 1), dtype=np.float32)
        self.sigma = np.eye(3, dtype=np.float32) * 0.01
        self.camera_intrinsics = camera_intrinsics
        self.camera_matrix = self.camera_intrinsics.get_camera_matrix()
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)  # Assuming no lens distortion
    
    def detect_aruco_markers(self, rgb_img):
        # Convert the image to grayscale
        gray = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
    
            marker_length = 0.5  # for example, 15 cm marker

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, self.camera_matrix, self.dist_coeffs)
            return_markers = []
            for i in range(len(ids)):
                rvec = rvecs[i][0]  # Rotation vector
                tvec = tvecs[i][0]  # Translation vector
                pts = corners[i][0]
                centre = pts.mean(axis=0)
                return_markers.append({
                    'id':ids[i][0], 
                    'rvec': rvec,
                    'tvec': tvec,
                    'corners_px': pts,
                    'centre_px': centre})   
                # print(f"Marker ID: {ids[i][0]}, rvec: {rvec}, tvec: {tvec}, corners: {pts}, centre: {centre}")        
            return return_markers
        else:
            return None
               