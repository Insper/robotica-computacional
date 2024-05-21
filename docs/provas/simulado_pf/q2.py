import time
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist, Point
# Adicione aqui os imports necessários
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
from my_package.goto import GoTo
from my_package.odom import Odom
from my_package.laser import Laser
import random


class Aleatorio(Node, Odom, Laser):

    def __init__(self):
        Node.__init__(self,'aleatorio_node')
        Odom.__init__(self)
        Laser.__init__(self)
        
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

        self.cor = {
            'amarelo':{
                'lower':(20,50,50),
                'upper':(40,255,255)
            },
            'magenta':{
                'lower':(140,50,50),
                'upper':(160,255,255)
            },
        }
        self.robot_state = 'gira'
        self.state_machine = {
            'gira': self.gira,
            'aproxima':self.aproxima,
            'afasta':self.afasta,
            'vorta':self.vorta,
            'stop': self.stop
        }

        self.tempo_aleatorio = random.randint(5,10)
        self.atual = False
        self.kp = 0.005
        self.vai_ate = GoTo(point=Point())


        # Inicialização de variáveis
        self.twist = Twist()
        self.cx = np.inf

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def image_callback(self, msg):
        self.cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8") # if CompressedImage
        h,w,_ = self.cv_image.shape
        self.w = w/2

        self.results = {
            'amarelo': self.processa_imagem(self.cor['amarelo']),
            'magenta': self.processa_imagem(self.cor['magenta']),
        }

        cv2.imshow('image', self.cv_image)
        cv2.waitKey(1)
    
    def processa_imagem(self,cor):
        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        
        mask = cv2.inRange(hsv, cor['lower'], cor['upper'])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            contour = max(contours, key=cv2.contourArea)
            cv2.drawContours(self.cv_image, contour, -1, [255, 0, 0], 3)

            M = cv2.moments(contour)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            cv2.circle(self.cv_image, (cx, cy), 5, (0, 0, 255), -1)

            return cx, cv2.contourArea(contour)
        else:
            return -1, -1

    def gira(self):
        if not self.atual:
            # get time current time in seconds
            self.atual = self.get_clock().now().to_msg().sec

        self.twist.angular.z = 0.5

        # time elapsed in seconds
        time_elapsed = self.get_clock().now().to_msg().sec - self.atual
        print('time_elapsed: ', time_elapsed)
        if time_elapsed >= self.tempo_aleatorio:
            if self.results['amarelo'][1] > self.results['magenta'][1]:
                self.robot_state = 'afasta'
                self.cor_selecionada = 'amarelo'
            else:
                self.robot_state = 'aproxima'
                self.cor_selecionada = 'magenta'

            self.vai_ate.point = Point(x=self.x, y=self.y)

    def calc_erro(self):
        self.erro = self.w - self.results[self.cor_selecionada][0]
        self.rot = self.erro * self.kp
        print('Erro Angular:', self.erro)
   
    def aproxima(self):
        self.calc_erro()
        self.twist.linear.x = 0.2
        self.twist.angular.z = self.rot

        if np.min(self.front) < 0.4:
            self.robot_state = 'vorta'

    def afasta(self):
        self.calc_erro()
        self.twist.linear.x = -0.2
        self.twist.angular.z = self.rot

        if np.min(self.back) < 0.4:
            self.robot_state = 'vorta'

    def vorta(self):
        if not self.vai_ate.robot_state == 'stop':
            print('Gotoing')
            rclpy.spin_once(self.vai_ate) # Roda o controle do rotate_node uma ve
            self.twist = self.vai_ate.twist
        else:
            self.robot_state = 'stop'
    
    def stop(self):
        self.twist = Twist()
        print(self.cor_selecionada)

    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()
        self.cmd_vel_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    ros_node = Aleatorio()

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()