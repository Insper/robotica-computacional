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
from my_package.segue_linha_p import Seguidor
from my_package.odom import Odom
from my_package.laser import Laser
from my_package.rotate2 import Rotate2
from my_package.goto import GoTo
import time

class Circuito(Node, Odom, Laser):

    def __init__(self, cor):
        Node.__init__(self,'circuito_node')
        Odom.__init__(self)
        Laser.__init__(self)
        time.sleep(2) # Bootando
        
        self.bridge = CvBridge()
        # self.cyellow = {
        #     'lower': (20, 50, 50),
        #     'upper': (30, 255, 255)
        # }
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
            'rotate':self.rotate,
            'vai_pa_caixa':self.vai_pa_caixa,
            'vorta':self.vorta,
            'stop': self.stop,
        }

        self.cor = {
            'vermelho':{
                'lower':(0,50,50),
                'upper':(20,255,255)
            },
            'verde':{
                'lower':(50,50,50),
                'upper':(80,255,255)
            },
            'azul':{
                'lower':(110,50,50),
                'upper':(120,255,255)
            },
        }
        self.cor = self.cor[cor]

        self.seguidor = Seguidor()
        self.seguidor.kp = 0.001
        self.seguidor.velx = 0.3
        self.kp = 0.001
        self.w = np.inf
        self.caixa_x = np.inf

        self.rotate_ate = Rotate2(90, deg=True)
        self.vai_ate = GoTo(point=Point())
        self.vai_ate.kp_angular = 0.5
        self.vai_ate.kp_linear = 0.4

        # Inicialização de variáveis
        self.twist = Twist()
        self.cx = np.inf
        self.bandeira = False
        self.front = [np.inf]
        self.x_inicial = np.inf

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8") # if CompressedImage
        h,w,_ = cv_image.shape
        self.w = w/2
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        mask = cv2.inRange(hsv, self.cor['lower'], self.cor['upper'])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            contour = max(contours, key=cv2.contourArea)
            cv2.drawContours(cv_image, contour, -1, [255, 0, 0], 3)

            M = cv2.moments(contour)
            self.caixa_x = int(M["m10"] / M["m00"])
            self.caixa_y = int(M["m01"] / M["m00"])

            cv2.circle(cv_image, (self.caixa_x, self.caixa_y), 5, (0, 0, 255), -1)
        else:
            return -1
        
    def distancia(self):
        return np.sqrt((self.x_inicial-self.x)**2+(self.y_inicial-self.y)**2)
    def segue(self):
        if self.x_inicial == np.inf:
            self.x_inicial = self.x
            self.y_inicial = self.y

        rclpy.spin_once(self.seguidor)
        self.twist = self.seguidor.twist
        distancia = self.distancia()

        if distancia < 0.1 and self.bandeira is True:
            self.robot_state = 'rotate'
        elif distancia > 0.2 and self.bandeira is False:
            self.bandeira = True
    
    def rotate(self):
        if not self.rotate_ate.robot_state == 'para':
            print('Rotating')
            rclpy.spin_once(self.rotate_ate) # Roda o controle do rotate_node uma ve
            self.twist = self.rotate_ate.twist
        else:
            self.robot_state = 'vai_pa_caixa'

    def calc_erro(self):
        self.erro = self.w - self.caixa_x
        self.rot = self.erro * self.kp
        print('Erro Angular:', self.erro)
        
    def vai_pa_caixa(self):
        self.calc_erro()
        self.twist.linear.x = 0.5
        self.twist.angular.z = self.rot 

        if np.min(self.front) < 0.5:
            self.robot_state = 'vorta'
            self.vai_ate.point = Point(x=self.x_inicial, y=self.y_inicial)
    
    def vorta(self):
        if not self.vai_ate.robot_state == 'stop':
            print('Gotoing')
            rclpy.spin_once(self.vai_ate) # Roda o controle do rotate_node uma ve
            self.twist = self.vai_ate.twist
        else:
            self.robot_state = 'stop'
    
    def stop(self):
        self.twist = Twist()

    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()
        self.cmd_vel_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    ros_node = Circuito('vermelho')

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()