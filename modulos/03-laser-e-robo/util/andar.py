import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robcomp_util.odom import Odom
from robcomp_util.laser import Laser

class Andar(Node, Odom): # Mude o nome da classe

    def __init__(self):
        super().__init__('andar_node') # Mude o nome do nó
        # Herda de Odom para obter funcionalidades de odometria
        Odom.__init__(self)

        # Placeholder para o timer
        self.timer = None

        # Definindo a maquina de estados
        self.robot_state = 'stop'
        self.state_machine = {
            'andar': self.andar,
            'stop': self.stop
        }

        # Inicialização de variáveis
        self.velocidade = 0.2

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
    
    def stop(self):
        # Zera a velocidade linear e angular
        self.twist = Twist()
        print("Andar: Parando o robô.")
        self.timer.cancel()
        self.timer = None
        self.done = True

    def reset(self):
        """Reseta o estado do robô e inicializa o timer."""
        # Zera a velocidade linear e angular
        self.twist = Twist()
        # Para indicar que a ação não foi concluída
        self.done = False
        # Muda para o estado inicial
        self.robot_state = 'andar'
        # Inicia o timer
        self.timer = self.create_timer(0.25, self.control)
        # Em seguida, inicia variaveis para realizar a ação
        # ...
        self.tempo_inicial = self.get_clock().now().to_msg()
        self.tempo_inicial = float(self.tempo_inicial.sec)

    def andar(self):
        # Robo deve andar pra frente por 4 segundos
        # e depois parar
        # ...
        self.twist.linear.x = self.velocidade
        self.tempo_atual = self.get_clock().now().to_msg()
        self.tempo_atual = float(self.tempo_atual.sec) 
        delta = self.tempo_atual - self.tempo_inicial
        print(f'Delta: {delta} segundos')

        if delta >= 4.0:
                self.twist.linear.x = 0.0
                self.robot_state = 'stop'


    def control(self):
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()     
        self.cmd_vel_pub.publish(self.twist)
 
def main(args=None):
    # Inicia o nó ROS2
    rclpy.init(args=args)
    # Cria uma instância do nó
    ros_node = Andar()

    # Ros2 spin_once para processar callbacks
    # Para capturar informações iniciais do Odom e Laser por exemplo
    rclpy.spin_once(ros_node)
    # Reset the node to initialize the goal yaw
    ros_node.reset()

    while not ros_node.robot_state == 'stop':
        rclpy.spin_once(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()