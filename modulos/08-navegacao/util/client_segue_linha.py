from geometry_msgs.msg import Point
from robcomp_interfaces.action import SimpleStart  # Import the correct action definition
import rclpy
from geometry_msgs.msg import Twist
from robcomp_util.odom import Odom
import numpy as np
from robcomp_util.client_base import BaseActionClientNode

class GoToActionClient(BaseActionClientNode, Odom):
    def __init__(self):
        """
        Inicializa o cliente de ação específico de 'GoToPoint'.
        """
        super().__init__('seguidor_linha_action_client', SimpleStart, 'segue_linha')
        Odom.__init__(self)

        self.pos_init = (self.x, self.y)
        self.hora_de_parar = False

        # Estado inicial da máquina de estados
        self.robot_state = 'segue'
        self.state_machine = {
            'segue': self.segue,
            'waiting_for_result': self.waiting_for_result,
            'stop': self.stop
        }

        # Cria um timer para controlar o robô
        self.timer = self.create_timer(0.25, self.control)

    def distance(self):
        """
        Calcula a distância entre dois pontos.
        """
        return np.sqrt((self.x - self.pos_init[0])**2 + (self.y - self.pos_init[1])**2)

    def segue(self):
        """
        Envia um objetivo para iniciar o seguidor de linha.
        """
        self.get_logger().info('Enviando objetivo para seguir a linha...')
        goal_msg = SimpleStart.Goal()
        goal_msg.start = True
        self.send_goal(goal_msg)
        self.robot_state = 'waiting_for_result'

    def waiting_for_result(self):
        """
        Verifica se o objetivo foi concluído e muda para o estado 'stop' se terminado.
        """
        dist = self.distance()
        self.get_logger().info(f'Distância percorrida: {dist:.2f} metros')

        if self.hora_de_parar is False and dist > 0.6:
            self.get_logger().info('Robô atingiu 0.6 metros, preparado para parar...')
            self.hora_de_parar = True

        elif self.hora_de_parar is True and dist < 0.5:
            self.get_logger().info('Cancelando objetivo, o robô está perto demais.')
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()  # Envia o comando para cancelar o objetivo
            self.robot_state = 'stop'

    def stop(self):
        """
        Para o robô, publicando um comando de velocidade zero.
        """
        self.get_logger().info('Parando o robô...')
        self.twist = Twist()
        self.cmd_vel_pub.publish(self.twist)

    def control(self):
        """
        Executa a máquina de estados, chamando a função correspondente ao estado atual.
        """
        self.twist = Twist()
        self.state_machine[self.robot_state]()

def main(args=None):
    """
    Função principal que inicializa o ROS2, cria o Action Client específico
    e mantém o nó rodando até ser encerrado.
    """
    rclpy.init(args=args)
    ros_node = GoToActionClient()

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
