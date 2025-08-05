import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import numpy as np

class Odom():
    def __init__(self):
        self.x = 0
        self.y = 0

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        rclpy.spin_once(self) # Roda pelo menos uma vez para pegar os valores de x, y e front

    def odom_callback(self, data: Odometry):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        orientation = data.pose.pose.orientation

        _, _, self.yaw = self.euler_from_quaternion(orientation)

    def euler_from_quaternion(self, orientation):
            """
            Converts quaternion (w in last place) to euler roll, pitch, yaw
            Assumed quaternion format: [x, y, z, w]
            """
            x = orientation.x
            y = orientation.y
            z = orientation.z
            w = orientation.w

            sinr_cosp = 2 * (w * x + y * z)
            cosr_cosp = 1 - 2 * (x * x + y * y)
            roll = np.arctan2(sinr_cosp, cosr_cosp)

            sinp = 2 * (w * y - z * x)
            pitch = np.arcsin(sinp)

            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            yaw = np.arctan2(siny_cosp, cosy_cosp)

            return roll, pitch, yaw