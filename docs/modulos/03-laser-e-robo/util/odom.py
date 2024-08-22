import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile

class Odom():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.yaw = 0

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

    def euler_from_quaternion(self, quaternion):
            """
            Converts quaternion (w in last place) to euler roll, pitch, yaw
            quaternion = [x, y, z, w]
            Below should be replaced when porting for ROS2 Python tf_conversions is done.
            """
            x = quaternion[0]
            y = quaternion[1]
            z = quaternion[2]
            w = quaternion[3]

            sinr_cosp = 2 * (w * x + y * z)
            cosr_cosp = 1 - 2 * (x * x + y * y)
            roll = np.arctan2(sinr_cosp, cosr_cosp)

            sinp = 2 * (w * y - z * x)
            pitch = np.arcsin(sinp)

            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            yaw = np.arctan2(siny_cosp, cosy_cosp)

            return roll, pitch, yaw

    def odom_callback(self, data: Odometry):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        quaternion = [
             data.pose.pose.orientation.x,
             data.pose.pose.orientation.y,
             data.pose.pose.orientation.z,
             data.pose.pose.orientation.w,
        ]

        _, _, self.yaw = self.euler_from_quaternion(quaternion)