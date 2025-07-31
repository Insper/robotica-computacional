import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robcomp_util.odom import Odom
from robcomp_util.laser import Laser

class Andar(Node, Odom, Laser): # Mude o nome da classe

    def __init__(self):
        super().__init__('andar_node') # Mude o nome do nó
        Odom.__init__(self)
        Laser.__init__(self)
        self.timer = self.create_timer(0.25, self.control)

        self.robot_state = 'andar'
        self.state_machine = {
            'andar': self.andar,
            'stop': self.stop
        }

        # Inicialização de variáveis

        self.velocidade = 0.2
        self.reset()
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
    
    def reset(self):
        self.twist = Twist()
        self.tempo_inicial = self.get_clock().now().to_msg()
        self.tempo_inicial = float(self.tempo_inicial.sec)

    def andar(self):
        self.twist.linear.x = self.velocidade
        self.tempo_atual = self.get_clock().now().to_msg()
        self.tempo_atual = float(self.tempo_atual.sec) 
        delta = self.tempo_atual - self.tempo_inicial
        print(f'Delta: {delta} segundos')

        if delta >= 2.0:
                self.twist.linear.x = 0.0
                self.robot_state = 'stop'

    def stop(self):
        self.twist = Twist()
        print("Parando o robô.")

    def control(self):
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()     
        self.cmd_vel_pub.publish(self.twist)
 
def main(args=None):
    rclpy.init(args=args)
    ros_node = Andar()

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()