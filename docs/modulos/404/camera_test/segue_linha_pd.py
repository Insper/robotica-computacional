import time
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from robcomp_interfaces.msg import AprilTagInsper, AprilTagInsperArray
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
        self.image = None
        self.subcomp = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.apriltag = self.create_subscription(
            AprilTagInsperArray,
            '/april_tags',
            self.apriltag_callback,
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
        self.kp = 0.003
        self.kd = 0.001  
        self.last_erro = 0 
        self.last_time = time.time() 

        self.twist = Twist()
        self.cx = np.inf

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def apriltag_callback(self, msg):
        if self.image is None:
            return
        
        # write id and draw a circle in the center of the tag
        for tag in msg.tags:
            cx = int(tag.cx)
            cy = int(tag.cy)
            cv2.circle(self.image, (cx, cy), 5, (0, 255, 0), -1)
            cv2.putText(self.image, str(tag.id), (cx + 10, cy + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imshow("Image with AprilTag", self.image)

    def image_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8") # if CompressedImage
        self.image = cv_image.copy()
        h,w,_ = cv_image.shape
        self.w = w/2

        # adjust brightness and contrast
        cv2.imshow("cv_image_before", cv_image)
        # cv_image = cv2.convertScaleAbs(cv_image, alpha=1.5, beta=0)
        # cv2.imshow("cv_image_after", cv_image)

        

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        mask = cv2.inRange(hsv, (20,50,90), (55,255,255))
        mask[:int(h/3),:] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cv2.imshow("mask", mask)
        cv2.waitKey(1)
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
        current_time = time.time()
        dt = current_time - self.last_time

        self.erro = self.w - self.cx
        
        # Termo Proporcional
        rot_p = self.erro * self.kp / 2

        # Termo Derivativo
        rot_d = 0
        if dt > 0:
            erro_derivativo = (self.erro - self.last_erro) / dt
            rot_d = erro_derivativo * self.kd

        self.rot = rot_p + rot_d 
        print(f'Erro Angular: {self.erro:.2f}, Termo Proporcional: {rot_p:.4f}, Termo Derivativo: {rot_d:.4f}, Rotação Total: {self.rot:.4f}')

        self.last_erro = self.erro 
        self.last_time = current_time 
        
    def segue(self):
        if self.cx == np.inf:
            if self.last_erro == 0:
                self.twist.angular.z = -0.3
            else:
                self.twist.angular.z = self.last_erro * self.kp
        else:
            self.calc_erro()
            self.twist.linear.x = 0.3
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