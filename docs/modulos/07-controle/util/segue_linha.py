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
        self.yellow = {
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
        self.timer = self.create_timer(0.2, self.control)

        self.robot_state = 'centraliza'
        self.state_machine = {
            'centraliza': self.centraliza,
            'segue': self.segue
        }

        self.threshold = 5

        # Inicialização de variáveis
        self.twist = Twist()
        self.x = np.inf

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8") # if CompressedImage
        h,w,_ = cv_image.shape
        self.w = w/2
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        mask = cv2.inRange(hsv, self.yellow['lower'], self.yellow['upper'])
        mask[:int(h/2),:] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            contour = max(contours, key=cv2.contourArea)
            cv2.drawContours(cv_image, contour, -1, [255, 0, 0], 3)

            M = cv2.moments(contour)
            self.x = int(M["m10"] / M["m00"])
            self.y = int(M["m01"] / M["m00"])

            cv2.circle(cv_image, (self.x, self.y), 5, (0, 0, 255), -1)

            cv2.imshow("cv_image", mask)
            cv2.waitKey(1)
        else:
            return -1

    def centraliza(self):
        erro = self.w - self.x
        print('Erro Angular:', erro)

        if abs(erro) < self.threshold:
            self.robot_state = 'segue'
            self.twist.angular.z = 0.0
        elif erro > 0:
            self.twist.angular.z = 0.1
        else:
            self.twist.angular.z = -0.1
        
    def segue(self):
        erro = self.w - self.x
        print('Erro Angular:', erro)

        self.twist.linear.x = 0.2
        if abs(erro) > self.threshold:
            self.twist.linear.x = 0.0
            self.robot_state = 'centraliza'

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