import math
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import numpy as np
import rclpy

class AMCL(): # Mude o nome da classe

    def __init__(self):
        # Subscribers
        self.odom_ready = False

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        # odom
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile)
        # tf
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            qos_profile)
        
        while not self.odom_ready:
            rclpy.spin_once(self)
            print("odom: retrying")
        print("Odom ready")

    def euler_from_quaternion(self, quaternion : list):
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

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        # Get the yaw (rotation around z-axis) from the quaternion
        _, _, self.odom_yaw = self.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def tf_callback(self, msg):
        # frame id needs to be /map
        for tf in msg.transforms:
            if tf.child_frame_id == 'odom':
                try:
                    # Translation
                    x_map = tf.transform.translation.x
                    y_map = tf.transform.translation.y

                    # Rotation (orientation in the map frame)
                    orientation_q = tf.transform.rotation
                    _, _, yaw_map = self.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

                    self.yaw = yaw_map + self.odom_yaw
                    self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

                    # Find the estimated robot pose in the map frame using tf and odom
                    delta_x = self.odom_x * math.cos(yaw_map) - self.odom_y * math.sin(yaw_map)
                    delta_y = self.odom_x * math.sin(yaw_map) + self.odom_y * math.cos(yaw_map)

                    self.x = x_map + delta_x
                    self.y = y_map + delta_y
                    self.get_logger().info(f'Current pose: ({self.x:.2f}, {self.y:.2f})')

                    self.start = self.world_to_map(self.x, self.y)
                    self.odom_ready = True
                except Exception as e:
                    self.get_logger().error(f"Error in tf_callback: {e}")