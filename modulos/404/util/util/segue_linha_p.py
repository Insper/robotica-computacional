import time
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
# Adicione aqui os imports necessários
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge

class Seguidor(Node):

    def __init__(self):
        super().__init__('seguidor_node')
        
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
        time.sleep(1)
        self.timer = self.create_timer(0.1, self.control)

        self.robot_state = 'segue'
        self.state_machine = {
            'segue': self.segue,
            'stop': self.stop
        }

        self.threshold = 5
        self.kp = 0.005

        # Inicialização de variáveis
        self.twist = Twist()
        self.cx = np.inf

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8") # if CompressedImage
        h,w,_ = cv_image.shape
        self.w = w/2
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        mask = cv2.inRange(hsv, self.cyellow['lower'], self.cyellow['upper'])
        mask[:int(h/2),:] = 0
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

            cv2.imshow("cv_image", mask)
            cv2.waitKey(1)
        else:
            return -1

    def calc_erro(self):
        self.erro = self.w - self.cx
        self.rot = self.erro * self.kp
        print('Erro Angular:', self.erro)
        
    def segue(self):
        if self.cx == np.inf:
            self.twist.angular.z = -0.4
        else:
            self.calc_erro()
            self.twist.linear.x = 0.5
            self.twist.angular.z = self.rot
    
    def stop(self):
        self.twist = Twist()

    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()
        self.cmd_vel_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    ros_node = Seguidor()

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()