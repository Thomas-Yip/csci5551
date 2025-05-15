#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import TwistStamped, PoseStamped
import numpy as np
import cv2
from camera_intrinsics import CameraIntrinsics
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from aruco_marker import ArucoMarker
from tf_transformations import euler_from_quaternion


MAX_FEATURES = 16
MAX_LANDMARKS = 250
DESC_DIM      = 32   # ORB’s descriptor length
ALPHA = 0.5
BETA = 0.2
GAMMA = 0.2
class EKFSLAM:
    def __init__(self, parent_node, rgb_topic, depth_topic, camera_intrinsics, qos):
        self.node = parent_node
        self.bridge = CvBridge()
        self.aruco_marker = ArucoMarker(camera_intrinsics)

        self.mu = np.zeros((3, 1), dtype=np.float32)  # State vector
        self.Fr = np.eye(3, dtype=np.float32)  # State transition matrix
        self.Q = np.eye(3, dtype=np.float32) * 0.1  # Process noise covariance
        self.sigma_rr = np.eye(3, dtype=np.float32) * 0.1  # Measurement noise covariance
        self.sigma_rm = None  # Cross-covariance
        self.sigma_mr = None
        self.sigma_mm = None  # Landmark covariance

        self.orb = cv2.ORB_create(nfeatures=MAX_FEATURES)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        self.num_landmarks = 0
        self.landmarks_pts = np.zeros((MAX_LANDMARKS, 3), dtype=np.float32) # (MAX_LANDMARKS, 3)
        self.landmark_kps  = [None] * MAX_LANDMARKS
        self.landmark_desc = np.zeros((MAX_LANDMARKS, DESC_DIM), dtype=np.uint8)
        
        self.observation = np.zeros((MAX_LANDMARKS, 1), dtype=np.int)
        self.responses = np.zeros((MAX_LANDMARKS, 1), dtype=np.float32)  
        self.scores = np.zeros((MAX_LANDMARKS, 1), dtype=np.float32)
        self.landmark_covariance = np.ones((MAX_LANDMARKS, 1), dtype=np.float32) * 10
        
        # self.trans_mat = np.array([[0, -1, 0, 0],
        #                             [-1, 0, 0, 0],
        #                             [0, 0, 1, -self.alt]],dtype=np.float32)

        # self.camera_intrinsics = CameraIntrinsics
  
        self.rgb_img = np.zeros((360, 640, 3), dtype=np.uint8)
        self.rgb_img_timestamp = None
        self.depth_img = np.zeros((360, 640), dtype=np.float32)
        self.depth_img_timestamp = None
        self.debug_img = np.zeros((360, 640, 3), dtype=np.uint8)
        
        self.last_vel_ts = None
        
        self.fx = camera_intrinsics.fx
        self.fy = camera_intrinsics.fy
        self.cx = camera_intrinsics.cx
        self.cy = camera_intrinsics.cy
        
        self.rgb_sub = self.node.create_subscription(Image,rgb_topic,self.rgb_cb,qos)
        self.depth_sub = self.node.create_subscription(Image,depth_topic,self.depth_cb,qos)
        self.vel_sub = self.node.create_subscription(TwistStamped,'/mavros/local_position/velocity_local',self.vel_callback,qos)
        self.pose_sub = self.node.create_subscription(PoseStamped,'/mavros/local_position/pose',self.pose_callback,qos)

        self.yaw = 0.0
        self.alt = 0.0

        self.R_bw = None
        self.q = None
        
    def vel_callback(self, msg):
        if self.q is None:
            return 
        q = self.q 
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_vel_ts is None:
            self.last_vel_ts = ts
            return  # Skip first frame, because we have no delta_t yet
        dt = ts - self.last_vel_ts

        if dt > 0:
            self.R_bw = np.array([
                [ np.cos(self.yaw), -np.sin(self.yaw)],
                [ np.sin(self.yaw),  np.cos(self.yaw)],
            ])
            vx = msg.twist.linear.x
            vy = msg.twist.linear.y
            v_body = np.array([vx, vy])
            v_world = self.R_bw @ v_body
            self.prediction(v_world[0], v_world[1], dt) # control input
        self.last_vel_ts = ts
    def rgb_cb(self, msg: Image):
        # Convert ROS Image to OpenCV
        self.rgb_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.rgb_img_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.measurement()

    def depth_cb(self, msg: Image):
        # Convert ROS Image to OpenCV
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        self.depth_img_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.measurement()          
    
    def pose_callback(self, msg: PoseStamped):
        self.alt = msg.pose.position.z
        self.q = msg.pose.orientation
        self.sigma_rr[2, 2] = 0.1
        if self.sigma_rm is not None:
            self.sigma_rm[2, :] = 0.0  # Reset cross-covariance from Z to all landmarks

        if self.sigma_mr is not None:
            self.sigma_mr[:, 2] = 0.0  # Reset cross-covariance from all landmarks to Z

    def prediction(self, vx, vy, dt):
        self.mu[0] += vx * dt
        self.mu[1] += vy * dt
        self.mu[2] = self.alt
        print("mu:", self.mu[:3])
        self.Fr = np.eye(3, dtype=np.float32)
        self.Fr[0, 2] = -vx * dt * np.sin(self.yaw)
        self.Fr[1, 2] = -vy * dt * np.cos(self.yaw)
        self.sigma_rr = self.Fr @ self.sigma_rr @ self.Fr.T + self.Q
        if(self.sigma_rm is not None):
            self.sigma_rm = self.Fr @ self.sigma_rm
            self.sigma_mr = self.sigma_rm.T    

    def measurement(self):
        if self.rgb_img_timestamp is None or self.depth_img_timestamp is None:
            print("rgb or depth image not received")
            return
        if(abs(float(self.rgb_img_timestamp) - float(self.depth_img_timestamp)) > 0.5):
            print("rgb and depth image timestamp not match")
            print(self.rgb_img_timestamp, self.depth_img_timestamp)
            return
        
        gray = cv2.cvtColor(self.rgb_img, cv2.COLOR_RGB2GRAY)

        if (np.sum(gray) == 0):
            print("rgb image is empty")
            print(gray)
            return
        kps, des = self.orb.detectAndCompute(gray, None)
        if len(kps) == 0:
            print("No keypoints detected")
            return
        if(self.R_bw is None):
            print("R_bw is None")
            return
        
        pts = np.array([kp.pt for kp in kps])
        responses = np.array([kp.response for kp in kps])
        sorted_ind = np.argsort(-responses)
        sorted_ind = sorted_ind[:2] # for debug use, don't delete
        pts = pts[sorted_ind]
        depth = self.depth_img[pts[:, 1].astype(int), pts[:, 0].astype(int)]
        projected_pts_cam_frame = self.back_project(pts[:, 0], pts[:, 1], depth)
        # print(self.R_bw.shape)
        # print(projected_pts_cam_frame[:2, :].shape)
        projected_pts_xy = self.R_bw @ projected_pts_cam_frame[:2, :]
        print(projected_pts_xy)
        tmp = projected_pts_xy[0, :].copy()
        projected_pts_xy[0, :] = -projected_pts_xy[1, :] + self.mu[0]
        projected_pts_xy[1, :] = tmp*-1 + self.mu[1]

        projected_pts_z = self.alt - projected_pts_cam_frame[2, :] 
        projected_pts_world = np.vstack((projected_pts_xy, projected_pts_z))
        responses = responses[sorted_ind]
        des = des[sorted_ind]
        kp = [kps[i] for i in sorted_ind]
        
        # add new landmarks
        num_new_landmarks = len(kp)
        self.debug_img = cv2.drawKeypoints(self.rgb_img, kp, None)

        if(self.num_landmarks != 0):

            matches = self.bf.knnMatch(self.landmark_desc[:self.num_landmarks, :], des, k=2)
            # print("Good matches:", len(good))
            good_matches = []
            for m,n in matches:
                if m.distance < 0.75 * n.distance:
                    good_matches.append(m)
            pts1_match = np.float32([ self.landmark_kps[m.queryIdx].pt for m in good_matches ]).reshape(-1,1,2)
            pts2_match = np.float32([ kps[m.trainIdx].pt for m in good_matches ]).reshape(-1,1,2)
            F, inliers = cv2.findFundamentalMat(pts1_match, pts2_match, cv2.FM_RANSAC, 3.0)
            
            num_inliers = int(np.count_nonzero(inliers))
            # for those matches that are inliers, update
            # else, add new landmarks
            # check if the number of inliers is greater than a threshold
            # print("Inlier matches:", num_inliers)
            # print("Matches:", len(good_matches))

            if (len(good_matches) > 0 and num_inliers/len(good_matches) > 0.2): # matches
                # update landmarks
                H = None
                z_measured_ind = []
                for i in range(len(good_matches)):
                    if (inliers[i] == 1):
                        idx = good_matches[i].queryIdx
                        z_measured_ind.append(good_matches[i].trainIdx)
                        R_cw = self.R_bw.T

                        # print(self.landmarks_pts[good_matches[i].trainIdx])
                        # print(projected_pts_cam_frame[:3, good_matches[i].trainIdx])
                        # di = self.landmarks_pts[good_matches[i].trainIdx] - projected_pts_cam_frame[:3, good_matches[i].trainIdx]
                        # h = np.hstack([-R_wc, np.zeros((3, 3*self.num_landmarks), dtype=np.float32)])
                        # h[:, 3*idx:3*idx+3] = 
                #         if H is None:
                #             H = h
                #         else:
                #             # H = np.vstack((H, h))
                        
                #         self.landmarks_pts[idx, :] = projected_pts.T[good_matches[i].trainIdx, :]
                #         self.landmark_kps[idx] = kp[good_matches[i].trainIdx]
                #         self.landmark_desc[idx, :] = des[good_matches[i].trainIdx]
                #         self.responses[idx, :] = responses[good_matches[i].trainIdx]
                #         self.observation[idx, :] += 1
                # if H is None:
                #     return
                # upper_sigma = np.hstack([self.sigma_rr, self.sigma_rm])
                # lower_sigma = np.hstack([self.sigma_mr, self.sigma_mm])
                # sigma = np.vstack((upper_sigma, lower_sigma))
                # R = np.eye(H.shape[0], dtype=np.float32) * 0.1
                # kalman_gain = sigma @ H.T @ np.linalg.inv(H @ sigma @ H.T + R)
                # mu = self.mu[:3]
                # # print(self.landmarks_pts.shape)
                # mu = np.vstack((mu, self.landmarks_pts[:self.num_landmarks].flatten().reshape(-1, 1)))
                # # mu = mu.reshape(-1, 1)
                # z_measured = projected_pts_cam_frame[:3, z_measured_ind].T.flatten().reshape(-1, 1)
                # mu = mu + kalman_gain @ (z_measured - H @ mu)
                # print(mu[:3])
                # # print("mu shape:", self.mu.shape)

                # sigma = (np.eye(sigma.shape[0]) - kalman_gain @ H) @ sigma
                # self.sigma_rr = sigma[:3, :3]
                # self.sigma_rm = sigma[:3, 3:]
                # self.sigma_mr = sigma[3:, :3]   
                # self.sigma_mm = sigma[3:, 3:]
                # # print("Landmarks updated")
                # return
                
        if (num_new_landmarks + self.num_landmarks < MAX_LANDMARKS) :
            self.add_landmark(projected_pts, kp, des, responses)
            # print("Landmarks added")
        # else:
            # print("Landmarks full")

    def add_landmark(self, projected_pts, kp, des, responses):
        num_new_landmarks = len(kp)
        self.landmarks_pts[self.num_landmarks:self.num_landmarks + num_new_landmarks, :] = projected_pts.T
        self.landmark_kps[self.num_landmarks:self.num_landmarks + num_new_landmarks] = kp
        self.landmark_desc[self.num_landmarks:self.num_landmarks + num_new_landmarks, :] = des
        self.responses[self.num_landmarks:self.num_landmarks + num_new_landmarks, :] = responses.reshape(-1, 1)
        self.observation[self.num_landmarks:self.num_landmarks + num_new_landmarks, :] += 1         

        if self.sigma_rm is None:
            self.sigma_rm = np.zeros((3, 3*num_new_landmarks), dtype=np.float32)
        else:
            new_sigma_rm = np.zeros((3, 3*(self.num_landmarks + num_new_landmarks)), dtype=np.float32)
            new_sigma_rm[:, :3*self.num_landmarks] = self.sigma_rm
            self.sigma_rm = new_sigma_rm
        
        if self.sigma_mr is None:
            self.sigma_mr = np.zeros((3*num_new_landmarks, 3), dtype=np.float32)
        else:
            new_sigma_mr = np.zeros((3*(self.num_landmarks + num_new_landmarks), 3), dtype=np.float32)
            new_sigma_mr[:3*self.num_landmarks, :] = self.sigma_mr
            self.sigma_mr = new_sigma_mr
        
        if self.sigma_mm is None:
            self.sigma_mm = np.eye(3*num_new_landmarks, dtype=np.float32) * 0.1
        else:
            new_sigma_mm = np.eye(3*(self.num_landmarks + num_new_landmarks), dtype=np.float32) * 0.1
            new_sigma_mm[:3*self.num_landmarks, :3*self.num_landmarks] = self.sigma_mm
            self.sigma_mm = new_sigma_mm
        
        self.num_landmarks += num_new_landmarks
        
    def back_project(self, u, v, d):
        X = (u - self.cx) * d / self.fx
        Y = (v - self.cy) * d / self.fy
        Z = d
        return np.array([X, Y, Z], dtype=np.float32)

    def update(self, measurement, measurement_noise):
        # Implement the update step of the EKF
        pass

    def get_state(self):
        return self.state

    def get_covariance(self):
        return self.covariance
    
    def get_debug_img(self):
        return self.debug_img