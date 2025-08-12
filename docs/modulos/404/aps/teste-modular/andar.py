import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Andar(Node,): # Mude o nome da classe

    def __init__(self):
        super().__init__('andar_node') # Mude o nome do nó
        self.timer = None

        self.robot_state = 'stop'
        self.state_machine = {
            'andar': self.andar,
            'stop': self.stop,
            'done': self.stop
        }

        # Inicialização de variáveis
        self.velocidade = 0.2

        # Publishers
        # Criação do publisher para o tópico que controla a velocidade do robô
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
    
    def reset(self):
        self.twist = Twist()
        self.robot_state = 'andar'
        if self.timer is None:
            self.timer = self.create_timer(0.25, self.control)
        ### Iniciar variaveis da ação
        # Tempo inicial
        self.tempo_inicial = self.get_clock().now().to_msg()
        self.tempo_inicial = float(self.tempo_inicial.sec)

    def andar(self):
        # Definar a velocidade linear do robô na variável twist
        self.twist.linear.x = self.velocidade
        # Recolher o tempo atual
        self.tempo_atual = self.get_clock().now().to_msg()
        self.tempo_atual = float(self.tempo_atual.sec)
        # Calcular o delta de tempo
        delta = self.tempo_atual - self.tempo_inicial
        print(f'Delta: {delta} segundos')
        # Se o delta é maior ou igual a t segundos, pare o robô
        if delta >= 4.0:
                self.twist.linear.x = 0.0
                self.robot_state = 'stop'

    def stop(self):
        self.twist = Twist()
        print("Parando o robô.")
        self.timer.cancel()
        self.timer = None
        self.robot_state = 'done'

    def control(self):
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()     
        self.cmd_vel_pub.publish(self.twist)
 
def main(args=None):
    rclpy.init(args=args)
    ros_node = Andar()

    rclpy.spin_once(ros_node)
    ros_node.reset()

    while not ros_node.robot_state == 'done':
        rclpy.spin_once(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()