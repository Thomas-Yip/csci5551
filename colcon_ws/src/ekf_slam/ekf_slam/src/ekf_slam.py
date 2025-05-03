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
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from tf_transformations import quaternion_matrix
import time
import csv

MAX_FEATURES = 16
MAX_LANDMARKS = 128
DESC_DIM      = 32   # ORB’s descriptor length

class EKFSLAM:
    def __init__(self, parent_node, rgb_topic, depth_topic, camera_intrinsics, qos):
        self.node = parent_node
        self.bridge = CvBridge()
        self.aruco_marker = ArucoMarker(camera_intrinsics)

        # State: [x, y, l1_x, l1_y, ..., lN_x, lN_y]^T
        self.mu = np.zeros((2, 1), dtype=np.float32)
        self.Sigma = np.eye(2, dtype=np.float32) * 0.1  # pose covariance

        # noise parameters
        self.Q_motion = np.eye(2, dtype=np.float32) * 0.1
        self.R_meas_var = 0.9
        self.init_landmark_cov = 1.0

        # ORB matcher
        self.orb = cv2.ORB_create(nfeatures=MAX_FEATURES)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        # landmarks storage
        self.num_landmarks = 0
        self.landmarks_pts = np.zeros((MAX_LANDMARKS, 2), dtype=np.float32)
        self.landmark_kps  = [None] * MAX_LANDMARKS
        self.landmark_desc = np.zeros((MAX_LANDMARKS, DESC_DIM), dtype=np.uint8)

        # image buffers
        self.rgb_img = np.zeros((360, 640, 3), dtype=np.uint8)
        self.depth_img = np.zeros((360, 640), dtype=np.float32)
        self.rgb_ts = None
        self.depth_ts = None

        # odometry
        self.last_odom = None

        # camera intrinsics
        self.fx = camera_intrinsics.fx
        self.fy = camera_intrinsics.fy
        self.cx = camera_intrinsics.cx
        self.cy = camera_intrinsics.cy

        # fixed transform robot<->world
        self.R_rw = np.array([[0, -1], [-1, 0]], dtype=np.float32)
        self.R_wr = self.R_rw.T

        # subscriptions
        self.node.create_subscription(Image, rgb_topic, self.rgb_cb, qos)
        self.node.create_subscription(Image, depth_topic, self.depth_cb, qos)
        self.node.create_subscription(Odometry, '/mavros/local_position/odom', self.odom_cb, qos)
        self.node.create_subscription(Imu, '/world/iris_runway/model/iris_with_depth_camera/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu', self.imu_cb, qos)
        self.pose_sub = self.node.create_subscription(PoseStamped,'/mavros/local_position/pose',self.pose_callback,qos)
        self.final_dest_sub = self.node.create_subscription(PoseStamped,'/drone/final_dest',self.final_dest_cb,qos)
        self.last_vel_ts = None
        self.last_imu_ts = None
        self.detected_markers = None
        self.yaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.debug_img = np.zeros((360, 640, 3), dtype=np.uint8)
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.pure_imu_estimate = np.zeros((3, 1), dtype=np.float32)
        self.pose = np.zeros((3, 1), dtype=np.float32)
        plt.ion()
        plt.show()

        self.history_slam = []   # EKF-SLAM poses
        self.history_imu = []    # pure IMU dead-reckoning
        self.history_gps = []    # IMU-GPS fused poses
        self.vel = np.zeros((3, 1), dtype=np.float32)
        self.once = False
        self.hikers = []
        self.final_dest = None

    def final_dest_cb(self, msg: PoseStamped):
        self.final_dest = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]).reshape(-1,1)
        # print("final_des:", self.final_des)

    def pose_callback(self, msg: PoseStamped):
        self.pose[0] = msg.pose.position.x
        self.pose[1] = msg.pose.position.y
        self.pose[2] = msg.pose.position.z
    
    def odom_cb(self, msg: Odometry):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if self.last_odom is None:
            self.last_odom = np.array([x, y], dtype=np.float32)
            return
        dx, dy = x - self.last_odom[0], y - self.last_odom[1]
        # self.prediction(dx, dy)
        self.last_odom = np.array([x, y], dtype=np.float32)

    def imu_cb(self, msg: Imu):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_imu_ts is None:
            self.last_imu_ts = ts
            return  # Skip first frame, because we have no delta_t yet
        dt = ts - self.last_imu_ts
        self.last_imu_ts = ts
        q = [msg.orientation.x,
             msg.orientation.y,
             msg.orientation.z,
             msg.orientation.w]
        R = quaternion_matrix(q)[:3, :3]
        g_body = R.dot(np.array([0.0, 0.0, 9.81]))
        a_raw = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z])
        a_lin = a_raw - g_body
        self.vel += a_lin.reshape(-1,1) * dt
        self.pure_imu_estimate[0] += self.vel[1] * dt
        self.pure_imu_estimate[1] += self.vel[0] * dt
        self.prediction(self.vel[1] * dt, self.vel[0] * dt) # control input

    def prediction(self, dx, dy):
        self.mu[0] += dx
        self.mu[1] += dy
        dim = self.Sigma.shape[0]
        Q_big = np.zeros((dim, dim), dtype=np.float32)
        Q_big[0:2, 0:2] = self.Q_motion
        self.Sigma += Q_big

    def rgb_cb(self, msg: Image):
        self.rgb_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.rgb_ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.try_measure()
        self.detect_hikers()
    
    def detect_hikers(self):
        hsv = cv2.cvtColor(self.rgb_img, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) > 100:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(self.rgb_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                d = float(self.depth_img[int(y), int(x)])
                p_robot = self.back_project(x, y, d)[:2]
                lm_w = (self.R_rw @ p_robot.reshape(2,1) + self.mu[0:2]).flatten()
                if len(self.hikers) == 0:
                    self.hikers.append(lm_w)
                else:
                    for i in range(len(self.hikers)):
                        hiker = self.hikers[i]
                        if np.linalg.norm(hiker - lm_w) < 1.0:
                            self.hikers[i] = (self.hikers[i] + lm_w) / 2
                cv2.putText(self.rgb_img, "Hiker: "+str(lm_w), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        print(self.hikers)        
        self.debug_img = self.rgb_img.copy()

    def depth_cb(self, msg: Image):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        self.depth_ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def try_measure(self):
        if self.rgb_ts is None or self.depth_ts is None:
            return
        if abs(self.rgb_ts - self.depth_ts) > 0.05:
            return
        
        threshold = 0.03490658503/2 # if < 2 degress, then do measurement
        
        if abs(self.roll) < threshold and abs(self.pitch) < threshold:
            self.measurement()

    def measurement(self):
        gray = cv2.cvtColor(self.rgb_img, cv2.COLOR_BGR2GRAY)
        tuple_kps, des = self.orb.detectAndCompute(gray, None)
        # self.debug_img = cv2.drawKeypoints(self.rgb_img, tuple_kps, None, color=(0, 255, 0), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        kps = list(tuple_kps)
        if kps is None or len(kps) == 0:
            return
    
        # initialise map if empty
        if self.num_landmarks == 0:
            self.add_landmarks(kps, des)
            return       
        matches = self.bf.knnMatch(self.landmark_desc[:self.num_landmarks], des, k=2)
        good = [m for m, n in matches if m.distance < 0.75 * n.distance]
    
        # 2) RANSAC on image correspondences
        pts1 = np.float32([self.landmark_kps[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        pts2 = np.float32([kps[m.trainIdx].pt              for m in good]).reshape(-1, 1, 2)
        _, mask = cv2.findFundamentalMat(pts1, pts2, cv2.FM_RANSAC, 3.0)
        inliers = [good[i] for i in range(len(good)) if mask is not None and mask[i, 0]]

        if len(inliers) == 0:
            return
   
        # 3) build stacked residual y and Jacobian H
        state_dim = self.Sigma.shape[0]
        m = len(inliers)                         # number of inlier landmarks
        H = np.zeros((2 * m, state_dim), np.float32)
        y = np.zeros((2 * m, 1),      np.float32)
        R_block = np.eye(2 * m,      dtype=np.float32) * self.R_meas_var

        matched_train = set()
        for j, m_obj in enumerate(inliers):
            qid, tid = m_obj.queryIdx, m_obj.trainIdx
            matched_train.add(tid)

            # 3-a) measurement in robot frame (pixel → metric)
            u, v = kps[tid].pt
            d = float(self.depth_img[int(v), int(u)])
            if d <= 0:
                continue
            z_meas = self.back_project(u, v, d)[:2].reshape(2, 1)

            # 3-b) predicted measurement from state
            lm_w   = self.landmarks_pts[qid].reshape(2, 1)
            delta  = lm_w - self.mu[0:2]             # world frame
            z_hat  = self.R_wr @ delta               # robot frame

            # 3-c) residual
            y[2*j:2*j+2, 0] = (z_meas - z_hat).flatten()

            # 3-d) Jacobian rows
            Hblock = np.hstack((-self.R_wr,          # ∂z/∂x, ∂z/∂y
                                np.zeros((2,1), np.float32)))  # no θ term (orientation fixed)
            H[2*j:2*j+2, 0:3] = Hblock
            idx = 2 + 2*qid                          # landmark start column in Σ
            H[2*j:2*j+2, idx:idx+2] = self.R_wr

        # 4) EKF update with all inliers at once
        S = H @ self.Sigma @ H.T + R_block
        K = self.Sigma @ H.T @ np.linalg.inv(S)
        self.mu    = self.mu + K @ y
        I          = np.eye(state_dim, dtype=np.float32)
        self.Sigma = (I - K @ H) @ self.Sigma
        self.Sigma = 0.5 * (self.Sigma + self.Sigma.T)   # numerical symmetry
        # self.update_plot(self.landmarks_pts[:self.num_landmarks].T)
        # print("num_landmarks:", self.num_landmarks)
        print('Robot pose relative to world frame: X, Y')
        print(f"IMU-RGBD EKF SlAM: {self.mu[:2,0]}")
        print(f'IMU-based Dead Reckoning: {self.pure_imu_estimate[:2,0]}')
        print(f'IMU-GPS-based position estimation: {self.pose[:2,0]}')
        print(f'num landmarks: {self.num_landmarks}')
        # after you update self.mu, self.pure_imu_estimate and self.pose:
        self.history_slam.append(self.mu[:2,0].copy())
        self.history_imu.append(self.pure_imu_estimate[:2,0].copy())
        self.history_gps.append(self.pose[:2,0].copy())
        # print('Error')
        # ------------------------------------------------------------
        # 5) add new landmarks that were not matched
        new_kps  = [kp  for i, kp  in enumerate(kps) if i not in matched_train]
        new_des  = [des for i, des in enumerate(des) if i not in matched_train]
        if new_kps:
            if self.num_landmarks == MAX_LANDMARKS:
                print("Too many landmarks, skipping measurement")
                # return
                self.queue_pruning()
            self.add_landmarks(new_kps, np.array(new_des, dtype=np.uint8))

        if (self.final_dest is None):
            return
        if abs(self.pose[0] - self.final_dest[0]) < 0.2 and abs(self.pose[1] - self.final_dest[1]) < 0.2 and not self.once:
            self.plot_trajectories_and_error()
            self.once = True
            # time.sleep(60)
    def random_pruning(self):
        remove_number = MAX_FEATURES * 2
        keep_ind = np.random.choice(self.num_landmarks, MAX_LANDMARKS - remove_number, replace=False).astype(np.int32)
        keep_ind = np.sort(keep_ind)
        print('keep ind:', keep_ind.shape)
        landmark_pts = np.zeros((MAX_LANDMARKS, 2), dtype=np.float32)
        landmark_kps = [None] * MAX_LANDMARKS
        landmark_desc = np.zeros((MAX_LANDMARKS, DESC_DIM), dtype=np.uint8)
        # print(self.landmark_kps.shape)
        landmark_kps[:MAX_LANDMARKS - remove_number] = [self.landmark_kps[i] for i in keep_ind]
        landmark_desc[:MAX_LANDMARKS - remove_number] = self.landmark_desc[keep_ind]
        landmark_pts[:MAX_LANDMARKS - remove_number] = self.landmarks_pts[keep_ind]
        mu = np.zeros((2 + (MAX_LANDMARKS - remove_number)*2, 1), dtype=np.float32)
        mu[:2] = self.mu[:2]
        mu_keep_ind = np.vstack(([keep_ind*2, keep_ind*2+1])).T.flatten()
        mu[2:] = self.mu[2 + mu_keep_ind]
        Sigma = np.zeros((2 + 2*(MAX_LANDMARKS - remove_number), 2 + 2*(MAX_LANDMARKS - remove_number)), dtype=np.float32)
        Sigma[:2, :2] = self.Sigma[:2, :2]
        Sigma[2:, 2:] = self.Sigma[2 + mu_keep_ind, 2 + mu_keep_ind]
        Sigma[:2, 2:] = self.Sigma[:2, 2 + mu_keep_ind]
        Sigma[2:, :2] = self.Sigma[2 + mu_keep_ind, :2]
        self.landmarks_pts = landmark_pts
        self.landmark_kps = landmark_kps
        self.landmark_desc = landmark_desc
        self.mu = mu
        self.Sigma = Sigma
        self.num_landmarks -= remove_number

    def queue_pruning(self):
        remove_number = MAX_FEATURES * 2
        landmark_pts = np.zeros((MAX_LANDMARKS, 2), dtype=np.float32)
        landmark_kps = [None] * MAX_LANDMARKS
        landmark_desc = np.zeros((MAX_LANDMARKS, DESC_DIM), dtype=np.uint8)
        landmark_kps[:MAX_LANDMARKS - remove_number] = self.landmark_kps[remove_number:]
        landmark_desc[:MAX_LANDMARKS - remove_number] = self.landmark_desc[remove_number:]
        landmark_pts[:MAX_LANDMARKS - remove_number] = self.landmarks_pts[remove_number:]
        mu = np.zeros((2 + (MAX_LANDMARKS - remove_number)*2, 1), dtype=np.float32)
        mu[:2] = self.mu[:2]
        mu[2:] = self.mu[2 + 2*remove_number:]
        Sigma = np.zeros((2 + 2*(self.num_landmarks - remove_number), 2 + 2*(self.num_landmarks - remove_number)), dtype=np.float32)
        Sigma[:2, :2] = self.Sigma[:2, :2]
        Sigma[2:, 2:] = self.Sigma[2 + 2*remove_number:, 2 + 2*remove_number:]
        self.landmarks_pts = landmark_pts
        self.landmark_kps = landmark_kps
        self.landmark_desc = landmark_desc
        self.mu = mu
        self.Sigma = Sigma
        self.num_landmarks -= remove_number
        
    def plot_trajectories_and_error(self):
        # Stack into (N×2) arrays
        slam_arr = np.vstack(self.history_slam)
        imu_arr  = np.vstack(self.history_imu)
        gps_arr  = np.vstack(self.history_gps)

        # 1) Mean Euclidean error between SLAM and IMU-GPS
        errors = np.linalg.norm(slam_arr - gps_arr, axis=1)
        mean_err = errors.mean()
        # print(f"Mean SLAM vs IMU-GPS error: {mean_err:.3f} units")
        with open('slam.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            for i in range(len(slam_arr)):
                writer.writerow([slam_arr[i, 0], slam_arr[i, 1]])
        with open('imu.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            for i in range(len(imu_arr)):
                writer.writerow([imu_arr[i, 0], imu_arr[i, 1]])
        with open('gps.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            for i in range(len(gps_arr)):
                writer.writerow([gps_arr[i, 0], gps_arr[i, 1]])

    def add_landmarks(self, kps, des):
        for kp, desc in zip(kps, des):
            if self.num_landmarks >= MAX_LANDMARKS:
                break
            u, v = kp.pt
            d = float(self.depth_img[int(v), int(u)])
            if d <= 0:
                continue
            p_robot = self.back_project(u, v, d)[:2]
            lm_w = (self.R_rw @ p_robot.reshape(2,1) + self.mu[0:2]).flatten()

            i = self.num_landmarks
            self.landmarks_pts[i] = lm_w
            self.landmark_kps[i] = kp
            self.landmark_desc[i] = desc

            # expand state
            self.mu = np.vstack([self.mu, lm_w.reshape(2,1)])
            # expand covariance
            old = self.Sigma
            old_dim = old.shape[0]
            new = np.zeros((old_dim+2, old_dim+2), dtype=np.float32)
            new[:old_dim,:old_dim] = old
            new[old_dim:,old_dim:] = np.eye(2, dtype=np.float32) * self.init_landmark_cov
            self.Sigma = new
            self.num_landmarks += 1

    def back_project(self, u, v, d):
        X = (u - self.cx) * d / self.fx
        Y = (v - self.cy) * d / self.fy
        return np.array([X, Y, d], dtype=np.float32)

    def update_plot(self, points3D):
        self.ax.clear()
        self.ax.scatter(points3D[0], points3D[1], 0, color='green')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        plt.draw()
        plt.pause(0.001)

    def inside_marker(self,pt, marker):
        # print('exe inside marker')
        c = marker["corners_px"]          
        # xmin, xmax = c[:, 0].min(), c[:, 0].max()
        # ymin, ymax = c[:, 1].min(), c[:, 1].max()
        # x, y = pt
        # return xmin <= x <= xmax and ymin <= y <= ymax
        return cv2.pointPolygonTest(c, pt, measureDist=False) >= 0
    def get_debug_img(self):
        return self.debug_img

    # def rm_marker_pts(self, )

# #!/usr/bin/env python3
# import rclpy
# from geometry_msgs.msg import TwistStamped, PoseStamped
# import numpy as np
# import cv2
# from camera_intrinsics import CameraIntrinsics
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image
# from sensor_msgs.msg import Imu
# from aruco_marker import ArucoMarker
# from nav_msgs.msg import Odometry
# from tf_transformations import euler_from_quaternion


# MAX_FEATURES = 16
# MAX_LANDMARKS = 250
# DESC_DIM      = 32   # ORB’s descriptor length
# ALPHA = 0.5
# BETA = 0.2
# GAMMA = 0.2
# class EKFSLAM:
#     def __init__(self, parent_node, rgb_topic, depth_topic, camera_intrinsics, qos):
#         self.node = parent_node
#         self.bridge = CvBridge()
#         self.aruco_marker = ArucoMarker(camera_intrinsics)

#         self.mu = np.zeros((3, 1), dtype=np.float32)  # State vector
#         self.Fr = np.eye(3, dtype=np.float32)  # State transition matrix
#         self.Q = np.eye(3, dtype=np.float32) * 0.1  # Process noise covariance
#         self.sigma_rr = np.eye(3, dtype=np.float32) * 0.1  # Measurement noise covariance
#         self.sigma_rm = None  # Cross-covariance
#         self.sigma_mr = None
#         self.sigma_mm = None  # Landmark covariance

#         self.orb = cv2.ORB_create(nfeatures=MAX_FEATURES)
#         self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

#         self.num_landmarks = 0
#         self.landmarks_pts = np.zeros((MAX_LANDMARKS, 2), dtype=np.float32) # (MAX_LANDMARKS, 3)
#         self.landmark_kps  = [None] * MAX_LANDMARKS
#         self.landmark_desc = np.zeros((MAX_LANDMARKS, DESC_DIM), dtype=np.uint8)
        
#         self.observation = np.zeros((MAX_LANDMARKS, 1), dtype=np.int)
#         self.responses = np.zeros((MAX_LANDMARKS, 1), dtype=np.float32)  
#         self.scores = np.zeros((MAX_LANDMARKS, 1), dtype=np.float32)
#         self.landmark_covariance = np.ones((MAX_LANDMARKS, 1), dtype=np.float32) * 10
  
#         self.rgb_img = np.zeros((360, 640, 3), dtype=np.uint8)
#         self.rgb_img_timestamp = None
#         self.depth_img = np.zeros((360, 640), dtype=np.float32)
#         self.depth_img_timestamp = None
#         self.debug_img = np.zeros((360, 640, 3), dtype=np.uint8)
        
#         self.last_vel_ts = None
        
#         self.fx = camera_intrinsics.fx
#         self.fy = camera_intrinsics.fy
#         self.cx = camera_intrinsics.cx
#         self.cy = camera_intrinsics.cy

#         self.cur_odom = np.zeros((3, 1), dtype=np.float32)
#         self.past_odom = None

#         self.rgb_sub = self.node.create_subscription(Image,rgb_topic,self.rgb_cb,qos)
#         self.depth_sub = self.node.create_subscription(Image,depth_topic,self.depth_cb,qos)
#         self.vel_sub = self.node.create_subscription(TwistStamped,'/mavros/local_position/velocity_local',self.vel_callback,qos)
#         self.pose_sub = self.node.create_subscription(PoseStamped,'/mavros/local_position/pose',self.pose_callback,qos)
#         self.imu_sub = self.node.create_subscription(Imu,'/mavros/imu/data',self.imu_callback,qos)
#         self.odom_sub = self.node.create_subscription(Odometry,'/mavros/local_position/odom',self.odom_callback,qos)
#         self.yaw = 0.0
#         self.alt = 0.0

#         self.R_bw = None
#         self.q = None
    
#     def odom_callback(self, msg: Odometry):
#         if(self.past_odom is None):
#             self.past_odom = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
#             return
#         self.cur_odom[0] = msg.pose.pose.position.x
#         self.cur_odom[1] = msg.pose.pose.position.y
#         self.cur_odom[2] = msg.pose.pose.position.z
#         dx = self.cur_odom[0] - self.past_odom[0]
#         dy = self.cur_odom[1] - self.past_odom[1]
#         self.prediction(dx, dy)

#         self.past_odom = self.cur_odom.copy()
        
    
#     def imu_callback(self, msg: Imu):
#         if self.q is None:
#             return 
#         q = msg.orientation
#         _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
#         # print("yaw imu:", self.yaw)

#     def vel_callback(self, msg):
#         if self.q is None:
#             return 
#         ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
#         if self.last_vel_ts is None:
#             self.last_vel_ts = ts
#             return  # Skip first frame, because we have no delta_t yet
#         dt = ts - self.last_vel_ts

#         if dt > 0:
#             self.R_bw = np.array([
#                 [ np.cos(self.yaw), -np.sin(self.yaw)],
#                 [ np.sin(self.yaw),  np.cos(self.yaw)],
#             ])
#             vx = msg.twist.linear.x
#             vy = msg.twist.linear.y
#             v_body = np.array([vx, vy])
#             # v_world = self.R_bw @ v_body
#             # self.prediction(v_body[0], v_body[1], dt) # control input
#         self.last_vel_ts = ts

#     def rgb_cb(self, msg: Image):
#         # Convert ROS Image to OpenCV
#         self.rgb_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         self.rgb_img_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
#         self.measurement()

#     def depth_cb(self, msg: Image):
#         # Convert ROS Image to OpenCV
#         self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
#         self.depth_img_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
#         self.measurement()          
    
#     def pose_callback(self, msg: PoseStamped):
#         self.alt = msg.pose.position.z
#         self.q = msg.pose.orientation
#         self.sigma_rr[2, 2] = 0.1
#         if self.sigma_rm is not None:
#             self.sigma_rm[2, :] = 0.0  # Reset cross-covariance from Z to all landmarks

#         if self.sigma_mr is not None:
#             self.sigma_mr[:, 2] = 0.0  # Reset cross-covariance from all landmarks to Z

#     def prediction(self, dx, dy):
#         self.mu[0] += dx
#         self.mu[1] += dy
#         # self.mu[2] = self.alt
#         print("mu:", self.mu[:2])
#         self.Fr = np.eye(3, dtype=np.float32)
#         self.sigma_rr = self.Fr @ self.sigma_rr @ self.Fr.T + self.Q
#         if(self.sigma_rm is not None):
#             self.sigma_rm = self.Fr @ self.sigma_rm
#             self.sigma_mr = self.sigma_rm.T    

#     def measurement(self):
#         if self.rgb_img_timestamp is None or self.depth_img_timestamp is None:
#             print("rgb or depth image not received")
#             return
#         if(abs(float(self.rgb_img_timestamp) - float(self.depth_img_timestamp)) > 0.5):
#             print("rgb and depth image timestamp not match")
#             print(self.rgb_img_timestamp, self.depth_img_timestamp)
#             return
        
#         gray = cv2.cvtColor(self.rgb_img, cv2.COLOR_RGB2GRAY)

#         if (np.sum(gray) == 0):
#             print("rgb image is empty")
#             print(gray)
#             return
#         kps, des = self.orb.detectAndCompute(gray, None)
#         if len(kps) == 0:
#             # print("No keypoints detected")
#             return
#         if(self.R_bw is None):
#             print("R_bw is None")
#             return
        
#         pts = np.array([kp.pt for kp in kps])
#         responses = np.array([kp.response for kp in kps])
#         sorted_ind = np.argsort(-responses)
#         sorted_ind = sorted_ind[:2] # for debug use, don't delete
#         pts = pts[sorted_ind]
#         depth = self.depth_img[pts[:, 1].astype(int), pts[:, 0].astype(int)]
#         proj_pts_robot_frame = self.back_project(pts[:, 0], pts[:, 1], depth)
#         trans_mat = np.array([[0, -1],
#                             [-1, 0]])
#         proj_pts_world_frame = trans_mat @ proj_pts_robot_frame[:2, :] + self.mu[:2].reshape(-1, 1)
#         # print('proj_pts_robot_frame:', proj_pts_robot_frame[:2])
#         # print("proj_pts_world_frame:", proj_pts_world_frame)

#         responses = responses[sorted_ind]
#         des = des[sorted_ind]
#         kp = [kps[i] for i in sorted_ind]
        
#         # add new landmarks
#         num_new_landmarks = len(kp)
#         self.debug_img = cv2.drawKeypoints(self.rgb_img, kp, None)

#         if(self.num_landmarks != 0):

#             matches = self.bf.knnMatch(self.landmark_desc[:self.num_landmarks, :], des, k=2)
#             # print("Good matches:", len(good))
#             good_matches = []
#             for m,n in matches:
#                 if m.distance < 0.75 * n.distance:
#                     good_matches.append(m)
#             pts1_match = np.float32([ self.landmark_kps[m.queryIdx].pt for m in good_matches ]).reshape(-1,1,2)
#             pts2_match = np.float32([ kps[m.trainIdx].pt for m in good_matches ]).reshape(-1,1,2)
#             F, inliers = cv2.findFundamentalMat(pts1_match, pts2_match, cv2.FM_RANSAC, 3.0)
            
#             num_inliers = int(np.count_nonzero(inliers))
#             # for those matches that are inliers, update
#             # else, add new landmarks
#             # check if the number of inliers is greater than a threshold
#             # print("Inlier matches:", num_inliers)
#             # print("Matches:", len(good_matches))

#             if (len(good_matches) > 0 and num_inliers/len(good_matches) > 0.2): # matches
#                 # update landmarks
#                 H = None
#                 z_measured_ind = []
#                 for i in range(len(good_matches)):
#                     if (inliers[i] == 1):
#                         idx = good_matches[i].queryIdx
#                         measured_id = good_matches[i].trainIdx

#                         z_expected = trans_mat.T @ (self.landmarks_pts[measured_id] - self.mu[:2].reshape(-1, 1))
#                         c_t = np.zeros((2, 2*self.num_landmarks), dtype=np.float32)
#                         c_t[:, 2*idx:2*idx+2] = np.eye(2, dtype=np.float32)
#                         c_t[:, 0:2] = trans_mat.T
#                         k = self.sigma @ c_t.T @ np.linalg.inv(c_t @ self.sigma @ c_t.T + self.Q)
#                         self.mu = self.mu + k @ (proj_pts_robot_frame[:2, idx] - z_expected)
#                         print(self.mu[:2])
                
#         if (num_new_landmarks + self.num_landmarks < MAX_LANDMARKS) :
#             self.add_landmark(proj_pts_world_frame, kp, des, responses)
#             # print("Landmarks added")
#         # else:
#             # print("Landmarks full")

#     def add_landmark(self, projected_pts, kp, des, responses):
#         num_new_landmarks = len(kp)
#         self.landmarks_pts[self.num_landmarks:self.num_landmarks + num_new_landmarks, :] = projected_pts.T
#         self.mu = np.hstack((self.mu, projected_pts.T))
#         self.landmark_kps[self.num_landmarks:self.num_landmarks + num_new_landmarks] = kp
#         self.landmark_desc[self.num_landmarks:self.num_landmarks + num_new_landmarks, :] = des
#         self.responses[self.num_landmarks:self.num_landmarks + num_new_landmarks, :] = responses.reshape(-1, 1)
#         self.observation[self.num_landmarks:self.num_landmarks + num_new_landmarks, :] += 1         

#         if self.sigma_rm is None:
#             self.sigma_rm = np.zeros((2, 2*num_new_landmarks), dtype=np.float32)
#         else:
#             new_sigma_rm = np.zeros((2, 2(self.num_landmarks + num_new_landmarks)), dtype=np.float32)
#             new_sigma_rm[:, :2*self.num_landmarks] = self.sigma_rm
#             self.sigma_rm = new_sigma_rm
#             self.sigma_mr = self.sigma_rm.T   
#         # if self.sigma_mr is None:
#         #     self.sigma_mr = np.zeros((2*num_new_landmarks, 2), dtype=np.float32)
#         # else:
#         #     new_sigma_mr = np.zeros((3*(self.num_landmarks + num_new_landmarks), 3), dtype=np.float32)
#         #     new_sigma_mr[:3*self.num_landmarks, :] = self.sigma_mr
#         #     self.sigma_mr = new_sigma_mr
        
#         if self.sigma_mm is None:
#             self.sigma_mm = np.eye(2*num_new_landmarks, dtype=np.float32) * 0.1
#         else:
#             new_sigma_mm = np.eye(2*(self.num_landmarks + num_new_landmarks), dtype=np.float32) * 0.1
#             new_sigma_mm[:2*self.num_landmarks, :2*self.num_landmarks] = self.sigma_mm
#             self.sigma_mm = new_sigma_mm
        
#         self.num_landmarks += num_new_landmarks
        
#     def back_project(self, u, v, d):
#         X = (u - self.cx) * d / self.fx
#         Y = (v - self.cy) * d / self.fy
#         Z = d
#         return np.array([X, Y, Z], dtype=np.float32)

#     def update(self, measurement, measurement_noise):
#         # Implement the update step of the EKF
#         pass

#     def get_state(self):
#         return self.state

#     def get_covariance(self):
#         return self.covariance
    
#     def get_debug_img(self):
#         return self.debug_img



# def measurement(self):
    #     gray = cv2.cvtColor(self.rgb_img, cv2.COLOR_BGR2GRAY)
    #     kps, des = self.orb.detectAndCompute(gray, None)
    #     if kps is None or len(kps) == 0:
    #         return
    #     # initialize landmarks
    #     if self.num_landmarks == 0:
    #         self.add_landmarks(kps, des)
    #         return

    #     # match descriptors
    #     matches = self.bf.knnMatch(self.landmark_desc[:self.num_landmarks], des, k=2)
    #     good = [m for m,n in matches if m.distance < 0.75 * n.distance]
    #     if not good:
    #         return
    #     # prepare for RANSAC
    #     pts1 = np.float32([ self.landmark_kps[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
    #     pts2 = np.float32([ kps[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
    #     F, mask = cv2.findFundamentalMat(pts1, pts2, cv2.FM_RANSAC, 3.0)
    #     if mask is None:
    #         inliers = []
    #     else:
    #         inliers = [good[i] for i in range(len(good)) if mask[i,0] == 1]

    #     matched_train = set()
    #     for m in inliers:
    #         matched_train.add(m.trainIdx)
    #         u, v = kps[m.trainIdx].pt
    #         d = float(self.depth_img[int(v), int(u)])
    #         if d <= 0:
    #             continue
    #         p_robot = self.back_project(u, v, d)[:2].reshape(2,1)

    #         # predicted measurement
    #         lm_w = self.landmarks_pts[m.queryIdx].reshape(2,1)
    #         delta = lm_w - self.mu[0:2]
    #         z_hat = self.R_wr @ delta
    #         z_meas = p_robot

    #         # print(f"Inlier landmark {m.queryIdx}: z_hat={z_hat.flatten()}, z_meas={z_meas.flatten()}")

    #         # build H
    #         state_dim = self.Sigma.shape[0]
    #         H = np.zeros((2, state_dim), dtype=np.float32)
    #         H[:,0:2] = -self.R_wr
    #         idx = 2 + 2*m.queryIdx
    #         H[:, idx:idx+2] = self.R_wr

    #         # EKF update
    #         Rm = np.eye(2) * self.R_meas_var
    #         S = H @ self.Sigma @ H.T + Rm
    #         K = self.Sigma @ H.T @ np.linalg.inv(S)
    #         y = z_meas - z_hat
    #         self.mu = (self.mu.flatten() + (K @ y).flatten()).reshape(-1,1)
    #         I = np.eye(state_dim, dtype=np.float32)
    #         self.Sigma = (I - K @ H) @ self.Sigma
    #         self.Sigma = (self.Sigma + self.Sigma.T) * 0.5
    #         print("num_landmarks:", self.num_landmarks)
    #         print(f"Post-update mu={self.mu[:2].flatten()}")

    #     # add new landmarks for unmatched keypoints
    #     new_kps = [kp for i,kp in enumerate(kps) if i not in matched_train]
    #     new_des = [des[i] for i in range(len(des)) if i not in matched_train]
    #     if new_kps:
    #         self.add_landmarks(new_kps, np.array(new_des))