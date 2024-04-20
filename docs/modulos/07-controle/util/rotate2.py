import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
# Adicione aqui os imports necessários
import numpy as np
import time
from my_package.odom import Odom

class RotateTo(Node, Odom): # Mude o nome da classe
    def __init__(self):
        Node.__init__(self, 'quadrado_node') # Mude o nome do nó
        Odom.__init__(self) # Mude o nome do nó
        time.sleep(1)
        self.timer = self.create_timer(0.25, self.control)

        self.robot_state = 'rotate'
        self.state_machine = {
            'stop': self.stop,
            'rotate': self.rotate
        }

        self.threshold = np.pi/180
        self.kp = 0.5

        # Inicialização de variáveis
        self.twist = Twist()

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
    
    def get_goal_from_target(self, ang, deg=True):
        """
        Inicializa o objetivo de rotação a partir de um ângulo alvo.
        Se deg=True, o ângulo é em graus, caso contrário, em radianos.
        """
        if deg:
            ang = np.radians(ang)

        self.goal_yaw = (ang + np.pi) % (2 * np.pi) - np.pi
        self.robot_state = 'rotate'
    
    def get_goal_from_rotation(self, rot, deg=True):
        """
        Inicializa o objetivo de rotação a partir de um ângulo de rotação.
        Angulo de rotação deve estar entre -pi e pi.
        Se deg=True, o ângulo é em graus, caso contrário, em radianos.
        """
        if deg:
            rot = np.radians(rot)

        new_yaw = self.yaw + rot
        self.goal_yaw = (new_yaw + np.pi) % (2 * np.pi) - np.pi
        self.robot_state = 'rotate'

    def get_angular_error(self):
        erro = self.goal_yaw - self.yaw
        self.erro = np.arctan2(np.sin(erro), np.cos(erro))

    def rotate(self):
        self.get_angular_error()
        if abs(self.erro) > self.threshold:
            self.twist.angular.z = self.kp * self.erro
        else:
            self.twist = Twist()
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
    ros_node = RotateTo()
    ros_node.get_goal_from_target(-45, deg=True)

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()