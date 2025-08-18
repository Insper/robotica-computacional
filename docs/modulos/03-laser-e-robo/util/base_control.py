import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist, Point
# Importar a classe da acao do arquivo, como por exemplo
from robcomp_util.base_action import Acao
# Adicione aqui os imports necessários

class BaseControlNode(Node): # Mude o nome da classe

    def __init__(self):
        super().__init__('base_control_node') # Mude o nome do nó
        # Outra Herança que você queira fazer
        rclpy.spin_once(self) # Roda pelo menos uma vez para pegar os valores
        self.acao_node = Acao() # Cria o nó da Acao

        self.robot_state = 'acao'
        self.state_machine = {
            'acao': self.acao, # Estado para GERENCIAR a ação
            'done': self.done
        }

        # Inicialização de variáveis
        self.twist = Twist()
        
        # Subscribers
        ## Coloque aqui os subscribers

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        ## Coloque aqui os publishers

        ## Por fim, inicialize o timer
        self.timer = self.create_timer(0.1, self.control)
    

    def acao(self):
        print("\nIniciando movimento de ação...")
        rclpy.spin_once(self.acao_node) # Processa as callbacks uma vez
        self.acao_node.reset() # Reseta o nó para iniciar a ação

        while not self.acao_node.robot_state == 'done': # Enquanto a ação não estiver finalizada
            rclpy.spin_once(self.acao_node) # Processa os callbacks e o timer

        # Quando a ação estiver finalizada, o 
        #   estado do robô deve ser alterado para o próximo estado ou finalizar mudando para 'done'
        self.robot_state = 'done'

    def done(self):
        self.twist = Twist()

    def control(self): # Controla a máquina de estados - eh chamado pelo timer
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]() # Chama o método do estado atual 
        self.cmd_vel_pub.publish(self.twist) # Publica a velocidade
 
def main(args=None):
    rclpy.init(args=args) # Inicia o ROS2
    ros_node = BaseControlNode() # Cria o nó

    while not ros_node.robot_state == 'done': # Enquanto o robô não estiver parado
        rclpy.spin_once(ros_node) # Processa os callbacks e o timer

    ros_node.destroy_node() # Destroi o nó
    rclpy.shutdown() # Encerra o ROS2

if __name__ == '__main__':
    main()