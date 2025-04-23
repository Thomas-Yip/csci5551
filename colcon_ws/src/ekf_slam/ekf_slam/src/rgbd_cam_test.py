#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class DepthSubscriber(Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        self.bridge = CvBridge()

        # Intrinsics from your SDF
        self.fx = 343.159
        self.fy = 343.159
        self.cx = 319.5
        self.cy = 179.5

        # Pixel coordinate you want to test (you can change these or make them params)
        self.u = 320
        self.v = 180

        self.sub = self.create_subscription(
            Image,
            '/rgbd_camera/depth_image',  # or '/camera/depth/image_raw'
            self.depth_cb,
            10)

    def depth_cb(self, msg: Image):
        # Convert ROS Image to OpenCV
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

        # Read the depth value (Z) at (u,v)
        d = float(depth[self.v, self.u])

        # Back‑project to camera frame
        X = (self.u - self.cx) * d / self.fx
        Y = (self.v - self.cy) * d / self.fy
        Z = d

        self.get_logger().info(
            f"Pixel ({self.u},{self.v}) → X={X:.3f} m, Y={Y:.3f} m, Z={Z:.3f} m"
        )

def main(args=None):
    rclpy.init(args=args)
    node = DepthSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
