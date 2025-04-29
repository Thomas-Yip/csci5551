#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
import cv2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from camera_intrinsics import CameraIntrinsics
from aruco_marker import ArucoMarker
from odom_sub import OdomSubscriber
from rgbd_sub import RGBDSubscriber
from ekf_slam import EKFSLAM

class Main(Node):
    def __init__(self):
        super().__init__('Main')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Can be BEST_EFFORT
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE  # Can be TRANSIENT_LOCAL
        )
        self.bridge = CvBridge()

        self.camera_intrinsics = CameraIntrinsics(334.159, 334.159, 319.5, 179.5)
    
        # self.rgbd_sub = RGBDSubscriber(
        #     self,
        #     self.camera_intrinsics,
        #     '/rgbd_camera/image',
        #     '/rgbd_camera/depth_image',
        #     qos_profile)
        
        self.odom_sub = OdomSubscriber(
            self,
            '/mavros/local_position/odom',
            qos_profile)
        
        # self.aruco_marker = ArucoMarker(self.camera_intrinsics)
    
        # self.sift = cv2.SIFT_create(nfeatures=64)
        self.orb = cv2.ORB_create()
        self.debug_pub = self.create_publisher(
            Image,
            '/rgbd_camera/debug',
            10)
        
        self.debug_img = np.zeros((360, 640, 3), dtype=np.uint8)

        self.ekf_slam = EKFSLAM(self, 
                                '/rgbd_camera/image',
                                '/rgbd_camera/depth_image',
                                self.camera_intrinsics, 
                                qos_profile)
        

        # Main loop
        self.timer = self.create_timer(0.02, self.main_loop)  # 10Hz
    
    def main_loop(self): 
        # Need to use rqt_image viewer to see the debug image
        # self.debug_pub.publish(self.bridge.cv2_to_imgmsg(self.debug_img, encoding='bgr8'))
        # print(self.rgbd_sub.get_works())
        # rgb_img, depth_img = self.rgbd_sub.get_rgbd_image()
        self.cur_odom, self.past_odom = self.odom_sub.get_odom()
        # self.ekf_slam.update_rgbd_img(rgb_img.copy(), depth_img.copy())
        # self.debug_img = self.ekf_slam.get_debug_img()
        # self.debug_pub.publish(self.bridge.cv2_to_imgmsg(self.debug_img, encoding='bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = Main()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
