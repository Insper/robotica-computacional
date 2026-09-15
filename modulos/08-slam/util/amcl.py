import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseWithCovarianceStamped


class AMCL(Node):
    def __init__(self):
        super().__init__('amcl_manual_pose_reader')

        # Pose final no frame map
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Pose no frame odom
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.has_odom = False
        self.pose_ready = False

        # QoS para /odom
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # QoS para /tf
        tf_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Subscribers
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            odom_qos,
        )

        self.tf_subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            tf_qos,
        )

        # Publisher da pose estimada no mapa
        self.amcl_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            10,
        )

        self.get_logger().info('Waiting for /odom and map -> odom transform...')

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def euler_from_quaternion(self, quaternion: list):
        """
        Converts quaternion [x, y, z, w] to roll, pitch, yaw.
        """
        x, y, z, w = quaternion

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        sinp = np.clip(sinp, -1.0, 1.0)
        pitch = np.arcsin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def quaternion_from_yaw(self, yaw):
        """
        Converte yaw em quaternion para movimento 2D no plano XY.
        """
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        return 0.0, 0.0, qz, qw

    def odom_callback(self, msg: Odometry):
        """
        Lê a pose do robô no frame odom.
        Normalmente: odom -> base_link.
        """
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation

        _, _, self.odom_yaw = self.euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ])

        self.has_odom = True

    def tf_callback(self, msg: TFMessage):
        """
        Procura o transform map -> odom e compõe com odom -> base_link.

        Resultado:
            self.x, self.y, self.yaw no frame map.

        Depois publica em /amcl_pose.
        """
        if not self.has_odom:
            return

        for tf in msg.transforms:
            parent_frame = tf.header.frame_id.lstrip('/')
            child_frame = tf.child_frame_id.lstrip('/')

            # Queremos especificamente map -> odom
            if parent_frame != 'map' or child_frame != 'odom':
                continue

            try:
                # Transformação map -> odom
                map_to_odom_x = tf.transform.translation.x
                map_to_odom_y = tf.transform.translation.y

                orientation_q = tf.transform.rotation

                _, _, map_to_odom_yaw = self.euler_from_quaternion([
                    orientation_q.x,
                    orientation_q.y,
                    orientation_q.z,
                    orientation_q.w,
                ])

                # Rotaciona a posição da odometria para o frame map
                rotated_odom_x = (
                    self.odom_x * math.cos(map_to_odom_yaw)
                    - self.odom_y * math.sin(map_to_odom_yaw)
                )

                rotated_odom_y = (
                    self.odom_x * math.sin(map_to_odom_yaw)
                    + self.odom_y * math.cos(map_to_odom_yaw)
                )

                # Pose final do robô no mapa
                self.x = map_to_odom_x + rotated_odom_x
                self.y = map_to_odom_y + rotated_odom_y
                self.yaw = self.normalize_angle(map_to_odom_yaw + self.odom_yaw)

                self.pose_ready = True

                self.publish_amcl_pose()

                self.get_logger().info(
                    f'Published /amcl_pose: '
                    f'x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f} rad'
                )

            except Exception as e:
                self.get_logger().error(f'Error in tf_callback: {e}')

    def publish_amcl_pose(self):
        """
        Publica a pose estimada no frame map no tópico /amcl_pose.
        """
        msg = PoseWithCovarianceStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = self.quaternion_from_yaw(self.yaw)

        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Covariância placeholder.
        # Ordem: x, y, z, roll, pitch, yaw em uma matriz 6x6 achatada.
        msg.pose.covariance = [0.0] * 36

        msg.pose.covariance[0] = 0.05   # variância em x
        msg.pose.covariance[7] = 0.05   # variância em y
        msg.pose.covariance[35] = 0.10  # variância em yaw

        self.amcl_pose_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = AMCL()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()