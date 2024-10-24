import time
import rclpy
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge
from robcomp_interfaces.action import SimpleStart
from robcomp_util.action_base import BaseActionServer

class SeguidorLinhaAction(BaseActionServer):

    def __init__(self):
        super().__init__('seguidor_action_server', SimpleStart, 'segue_linha')  # Use the base constructor
        
        self.bridge = CvBridge()
        self.cyellow = {
            'lower': (20, 50, 50),
            'upper': (30, 255, 255)
        }
        self.kernel = np.ones((10,10), np.uint8)
        self.subcomp = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.robot_state = 'segue'
        self.state_machine = {
            'segue': self.segue,
            'stop': self.stop
        }

        self.threshold = 5
        self.kp = 0.005
        self.velx = 0.5

        # Inicialização de variáveis
        self.twist = Twist()
        self.cx = np.inf

        self.timer = self.create_timer(0.1, self.control)
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        while rclpy.ok():
            rclpy.spin_once(self)
            if self.robot_state == 'stop':
                break

        self._result_msg.success = True
        goal_handle.succeed()

        return self._result_msg

    ### Segue Linha

    def image_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")  # if CompressedImage
        h, w, _ = cv_image.shape
        self.w = w / 2
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        mask = cv2.inRange(hsv, self.cyellow['lower'], self.cyellow['upper'])
        mask[:int(h/2), :] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            contour = max(contours, key=cv2.contourArea)
            cv2.drawContours(cv_image, contour, -1, [255, 0, 0], 3)

            M = cv2.moments(contour)
            self.cx = int(M["m10"] / M["m00"])
            self.cy = int(M["m01"] / M["m00"])

            cv2.circle(cv_image, (self.cx, self.cy), 5, (0, 0, 255), -1)

        else:
            self.cx = np.inf

    def calc_erro(self):
        self.erro = self.w - self.cx
        self.rot = self.erro * self.kp
        
    def segue(self):
        if self.cx == np.inf:
            self.twist.angular.z = -0.4
        else:
            self.calc_erro()
            self.twist.linear.x = self.velx
            self.twist.angular.z = self.rot
    
    def stop(self):
        self.twist = Twist()

    def control(self):
        self.twist = Twist()

        if self._goal_handle is not None:
            self.state_machine[self.robot_state]()
            self.cmd_vel_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    ros_node = SeguidorLinhaAction()

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
