
# https://codeshare.io/Ap7Qjr
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from robcomp_util.laser import Laser
from robcomp_util.odom import Odom
import time
import numpy as np
# Adicione aqui os imports necessários

class Fugitivo(Node, Laser, Odom): # Mude o nome da classe

    def __init__(self):
        super().__init__('fugitivo_node') # Mude o nome do nó
        Laser.__init__(self)
        Odom.__init__(self)
        self.timer = self.create_timer(0.25, self.control)

        self.robot_state = 'andar'
        self.state_machine = {
            'stop': self.stop,
            'andar': self.andar,
            'girar': self.girar,
        }

        # Inicialização de variáveis
        self.twist = Twist()
        
        # Subscribers
        ## Coloque aqui os subscribers

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.watcher_pub = self.create_publisher(String, 'watcher', 10)
        msg = String()
        msg.data = "stop"
        self.watcher_pub.publish(msg)
        msg = String()
        msg.data = "start"
        self.watcher_pub.publish(msg)

        self.apontador = 0
        self.giros = False

    def stop(self):
        self.twist = Twist()
        print(self.front)
    
    def andar(self):
        if not self.giros:
            if self.x > 0:
                self.giros = [-1, -1, 1, 1, -1]
            else:
                self.giros = [-1, 1, -1, -1, 1]
        self.twist.linear.x = 0.2

        if min(self.front) < 0.5:
            self.robot_state = "girar"
            self.goal_yaw = self.yaw + self.giros[self.apontador] * np.pi / 2
            self.apontador += 1

    def girar(self):
        erro = self.goal_yaw - self.yaw
        erro = np.arctan2(np.sin(erro), np.cos(erro))

        print(f'Erro: {erro}')
        if abs(erro) < np.deg2rad(1):
            self.robot_state = "andar"
        else:
            if erro < 0:
                self.twist.angular.z = -0.2
            else:
                self.twist.angular.z = 0.2


    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()

        self.cmd_vel_pub.publish(self.twist)
        
            
def main(args=None):
    rclpy.init(args=args)
    ros_node = Fugitivo() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()