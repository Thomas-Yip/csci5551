# odom_sub.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import numpy as np
class OdomSubscriber:
    def __init__(self, parent_node, topic, qos):
        self.node = parent_node
        
        self.odom_sub = self.node.create_subscription(
            Odometry,
            topic,
            self.odom_cb,
            qos)
        self.cur_odom = np.zeros((3, 1), dtype=np.float32)
        self.past_odom = np.zeros((3, 1), dtype=np.float32)

    def odom_cb(self, msg: Odometry):
        # Extract the position from the Odometry message
        self.past_odom = self.cur_odom.copy()
        self.cur_odom[0, 0] = msg.pose.pose.position.x
        self.cur_odom[1, 0] = msg.pose.pose.position.y
        self.cur_odom[2, 0] = msg.pose.pose.position.z
        # print("received odom data")

    def get_odom(self):
        return self.cur_odom, self.past_odom
    
