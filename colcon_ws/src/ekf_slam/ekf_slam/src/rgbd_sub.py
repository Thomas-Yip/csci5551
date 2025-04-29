# rgbd_sub.py
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from camera_intrinsics import CameraIntrinsics
class RGBDSubscriber:
    def __init__(self,parent_node, camera_intrinsics, rgb_topic: str, depth_topic: str, qos):
        # super().__init__('rgbd_subscriber')
        self.node = parent_node
        self.bridge = CvBridge()

        self.camera_intrinsics = camera_intrinsics

        self.rgb_sub = self.node.create_subscription(
            Image,
            rgb_topic,
            self.rgb_cb,
            qos)
        self.rgb_img = np.zeros((360, 640, 3), dtype=np.uint8)

        self.depth_sub = self.node.create_subscription(
            Image,
            depth_topic,
            self.depth_cb,
            qos)
        self.d_img = np.zeros((360, 640), dtype=np.float32)

    def rgb_cb(self, msg: Image):
        # Convert ROS Image to OpenCV
        self.rgb_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # print("received rgb image")

    def depth_cb(self, msg: Image):
        # Convert ROS Image to OpenCV
        self.d_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        # print("received depth image")

    def get_rgbd_image(self):
        return self.rgb_img, self.d_img
    