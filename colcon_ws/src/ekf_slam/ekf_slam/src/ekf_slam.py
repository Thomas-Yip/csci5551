#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
class DepthSubscriber(Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        self.bridge = CvBridge()
         # Intrinsics from your SDF
        self.fx = 343.159
        self.fy = 343.159
        self.cx = 319.5
        self.cy = 179.5

        self.rgb_sub = self.create_subscription(
            Image,
            '/rgbd_camera/image',  # or '/camera/depth/image_raw'
            self.rgb_cb,
            10)
        self.rgb_img = np.zeros((360, 640,3), dtype=np.uint8)

        self.depth_sub = self.create_subscription(
            Image,
            '/rgbd_camera/depth_image',  # or '/camera/depth/image_raw'
            self.depth_cb,
            10)
        self.d_img = np.zeros((360, 640), dtype=np.float32)

        self.sift = cv2.SIFT_create(nfeatures=64)

        self.debug_pub = self.create_publisher(
            Image,
            '/rgbd_camera/debug',
            10)
        self.debug_img = np.zeros((360, 640, 3), dtype=np.uint8)

        # Main loop
        self.timer = self.create_timer(0.1, self.main_loop)  # 10Hz

        # aruco marker detection
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        # detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    def depth_cb(self, msg: Image):
        # Convert ROS Image to OpenCV
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        self.img = depth
    
    def rgb_cb(self, msg: Image):
        # Convert ROS Image to OpenCV
        rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.rgb_img = rgb
    
    def main_loop(self): 
        # feature extraction 
        kp, des = self.sift.detectAndCompute(self.rgb_img, None)
        # draw keypoints
        rgb_img = cv2.drawKeypoints(self.rgb_img, kp, None)
        self.debug_img = rgb_img
        # print("Keypoints:", len(kp))
        self.detect_aruco_markers()
        # Need to use rqt_image viewer to see the debug image
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(self.debug_img, encoding='bgr8'))
    
    def detect_aruco_markers(self):
        # Convert the image to grayscale
        gray = cv2.cvtColor(self.rgb_img, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        print("Detected markers:", ids)

def main(args=None):
    rclpy.init(args=args)
    node = DepthSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
