#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_matrix
import numpy as np

class ImuIntegrator(Node):
    def __init__(self):
        super().__init__('imu_integrator')

        # QoS for a high‐rate sensor: best‐effort, keep last 10 msgs
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)

        # Subscribe to the Gazebo IMU topic
        self.imu_sub = self.create_subscription(
            Imu,
            '/world/iris_runway/model/iris_with_depth_camera/model/iris_with_standoffs/'
            'link/imu_link/sensor/imu_sensor/imu',
            self.imu_callback,
            sensor_qos)

        self.prev_time = None
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)

    def imu_callback(self, msg: Imu):
        # timestamp
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            self.prev_time = t
            return
        dt = t - self.prev_time
        self.prev_time = t

        # build rotation matrix from world→body
        q = [msg.orientation.x,
             msg.orientation.y,
             msg.orientation.z,
             msg.orientation.w]
        R = quaternion_matrix(q)[:3, :3]

        # gravity in body frame
        g_body = R.dot(np.array([0.0, 0.0, 9.81]))

        # raw accel
        a_raw = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z])

        # remove gravity
        a_lin = a_raw - g_body

        # integrate velocity & position
        self.velocity += a_lin * dt
        self.position += self.velocity * dt

        print(
            f"dt={dt:.4f}  a_lin={a_lin.round(3)}  vel={self.velocity.round(3)}  pos={self.position.round(3)}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ImuIntegrator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
