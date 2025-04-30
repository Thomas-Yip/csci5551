#!/usr/bin/env python3
import rclpy, cv2, numpy as np
from rclpy.node       import Node
from sensor_msgs.msg  import Image
from cv_bridge        import CvBridge
from collections      import deque

class CamStabiliser(Node):
    def __init__(self):
        super().__init__('cam_stabiliser')
        qos = 10
        self.sub = self.create_subscription(Image,
                                            '/rgbd_camera/image',
                                            self.cb, qos)
        self.pub = self.create_publisher(Image,
                                         '/rgbd_camera/stab',
                                         qos)
        self.bridge = CvBridge()
        self.prev_gray = None
        self.transforms = deque(maxlen=30)      # sliding window for smoothing
        self.accumulated = np.eye(3, dtype=np.float32)

    # ---------- parameters you can tune ----------
        self.alpha = 0.3     # 0   ← smoother  •  1 ← snappier
        self.max_corners = 150
        self.quality     = 0.01
        self.min_dist    = 7
        # ---------------------------------------------

    def cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.prev_gray is None:
            self.prev_gray = gray
            self.last_stamp = msg.header.stamp
            return

        # 1) detect + track corners
        p0 = cv2.goodFeaturesToTrack(self.prev_gray,
                                     self.max_corners,
                                     self.quality,
                                     self.min_dist)
        if p0 is None:
            self.prev_gray = gray
            return

        p1, st, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, p0, None)
        good_prev = p0[st == 1]
        good_curr = p1[st == 1]

        if len(good_prev) < 10:
            self.prev_gray = gray
            return

        # 2) estimate similarity (2×3)
        M, inliers = cv2.estimateAffinePartial2D(good_prev, good_curr,
                                                 method=cv2.RANSAC,
                                                 ransacReprojThreshold=3.0)
        if M is None:
            self.prev_gray = gray
            return

        # 3) turn to 3×3,  accumulate & smooth
        M33 = np.vstack([M, [0, 0, 1]]).astype(np.float32)
        self.accumulated = self.accumulated @ M33          # compose
        self.transforms.append(self.accumulated.copy())

        #    simple exponential moving average
        if len(self.transforms) >= 2:
            smoothed = self.alpha * self.transforms[-1] + \
                       (1 - self.alpha) * self.transforms[-2]
        else:
            smoothed = self.accumulated.copy()

        # 4) inverse warp
        H_inv = np.linalg.inv(smoothed)
        stab = cv2.warpPerspective(frame, H_inv, (frame.shape[1], frame.shape[0]),
                                   flags=cv2.INTER_LINEAR)

        # optional crop 5 % to hide black borders
        h, w = stab.shape[:2]
        c = int(0.05 * min(w, h))
        stab = stab[c:h-c, c:w-c]
        stab = cv2.resize(stab, (w, h))

        # 5) publish
        out = self.bridge.cv2_to_imgmsg(stab, 'bgr8')
        out.header.stamp  = msg.header.stamp
        out.header.frame_id = 'camera_stab'
        self.pub.publish(out)

        self.prev_gray = gray


def main():
    rclpy.init()
    node = CamStabiliser()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
