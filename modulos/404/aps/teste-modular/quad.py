import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from robcomp_util.odom import Odom
from robcomp_util.laser import Laser
# Adicione aqui os imports necessários

class Quadrado(Node, Odom, Laser): # Mude o nome da classe

    def __init__(self):
        super().__init__('quadrado_node') # Mude o nome do nó
        Odom.__init__(self)
        Laser.__init__(self)
        self.timer = self.create_timer(0.25, self.control)

        self.robot_state = 'andar'
        self.state_machine = {
            'andar': self.andar,
            'girar': self.girar
        }

        # Inicialização de variáveis
        self.twist = Twist()
        self.tempo_inicial = self.get_clock().now().to_msg()
        self.tempo_inicial = float(self.tempo_inicial.sec)
        self.velocidade = 0.2
        self.giro = 0.2
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
    

    def andar(self):
        self.twist.linear.x = self.velocidade
        self.tempo_atual = self.get_clock().now().to_msg()
        self.tempo_atual = float(self.tempo_atual.sec) 
        delta = self.tempo_atual - self.tempo_inicial
        print(f'Delta: {delta} segundos')

        if delta >= 2.0:
                self.twist.linear.x = 0.0

                self.robot_state = 'girar'
                self.goal_yaw = self.yaw + np.pi / 2 

    def girar(self):
        
        self.erro = self.goal_yaw - self.yaw 
        self.erro = np.arctan2(np.sin(self.erro), np.cos(self.erro)) 

        print(f'Erro: {np.degrees(self.erro)}')

        if self.erro > 0:
            self.twist.angular.z = self.giro
        else: 
            self.twist.angular.z = -self.giro

        if abs(self.erro) < np.radians(2):
                self.twist.angular.z = 0.0 
                
                self.tempo_inicial = self.get_clock().now().to_msg() 
                self.tempo_inicial = float(self.tempo_inicial.sec)
                self.robot_state = 'andar'

    def control(self):
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()     
        self.cmd_vel_pub.publish(self.twist)
 
def main(args=None):
    rclpy.init(args=args)
    ros_node = Quadrado()

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()