import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import CancelResponse, GoalResponse
from geometry_msgs.msg import Twist, Point
from robcomp_util.odom import Odom
from robcomp_interfaces.action import GoToPoint  # Importe sua ação

import numpy as np
import time

class GoToActionServer(Node, Odom):
    """
    Este nó implementa um Action Server para mover o robô até um ponto específico.
    Ele herda de Node e Odom, onde Node é o componente básico do ROS2 e Odom lida
    com informações de odometria.
    """

    def __init__(self):
        """
        Inicializa o Action Server, define parâmetros de controle e cria
        os publicadores e timers necessários.
        """
        Node.__init__(self, 'goto_action_server')
        Odom.__init__(self)

        # Cria o servidor de ação para o 'GoToPoint'
        self._action_server = ActionServer(
            self, # Nó
            GoToPoint, # Tipo da ação
            'goto_point', # Nome do tópico de ação
            execute_callback=self.execute_callback, # LOOP PRINCIPAL
            goal_callback=self.goal_callback, # Aceita ou rejeita o objetivo
            handle_accepted_callback=self.handle_accepted_callback, # Inicia a execução
            cancel_callback=self.cancel_callback # Lidar com pedidos de cancelamento
            )

        self._goal_handle = None
        self._result_msg = GoToPoint.Result()

        # Inicializa parâmetros de controle
        self.twist = Twist()
        self.threshold = np.pi / 180
        self.kp_linear = 0.8
        self.kp_angular = 0.5
        self.max_vel = 0.5

        # Define o estado inicial da máquina de estados
        self.robot_state = 'center'
        self.state_machine = {
            'center': self.center,
            'goto': self.goto,
            'stop': self.stop
        }

        # Cria um timer para controlar o robô
        self.timer = self.create_timer(0.25, self.control)

        # Publicador para enviar comandos de velocidade ao robô
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def goal_callback(self, goal_request):
        """
        Função chamada quando um novo objetivo é recebido.
        Retorna a aceitação do objetivo.
        """
        self.get_logger().info('Recebeu um novo objetivo')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """
        Função chamada quando o objetivo é aceito. Inicia a execução do objetivo.
        """
        self._goal_handle = goal_handle
        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        """
        Função chamada quando um cancelamento de objetivo é solicitado.
        Cancela a execução e para o robô.
        """
        self.get_logger().info('Recebeu um pedido de cancelamento')
        self._goal_handle = None
        goal_handle.abort()
        self.robot_state = 'stop'
        self.cmd_vel_pub.publish(Twist())
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """
        Função que executa a movimentação do robô até o ponto solicitado.
        Atualiza o estado do robô até atingir o objetivo ou ser interrompido.
        """
        self.get_logger().info('Iniciando movimento para o ponto')
        self.point = goal_handle.request.goal

        while rclpy.ok(): # Enquanto o ROS2 estiver rodando
            rclpy.spin_once(self)
            if self.robot_state == 'stop': # Finaliza quando o estado é 'stop'
                break

        self._result_msg.success = True
        goal_handle.succeed()
        self._goal_handle = None

        return self._result_msg

    def get_angular_error(self):
        """
        Calcula o erro angular entre a posição atual e o ponto objetivo.
        Ajusta a velocidade angular para reduzir o erro.
        """
        x = self.point.x - self.x
        y = self.point.y - self.y
        theta = np.arctan2(y, x)

        self.distance = np.sqrt(x ** 2 + y ** 2)
        erro = theta - self.yaw
        self.erro = np.arctan2(np.sin(erro), np.cos(erro))

        self.twist.angular.z = self.erro * self.kp_angular

    def center(self):
        """
        Controla o robô para alinhar sua direção com o ponto objetivo.
        Quando o erro angular é pequeno, muda para o estado 'goto'.
        """
        self.get_angular_error()

        if abs(self.erro) < np.deg2rad(3):
            self.robot_state = 'goto'

    def goto(self):
        """
        Move o robô em direção ao ponto objetivo.
        Quando a distância ao ponto é pequena, muda para o estado 'stop'.
        """
        self.get_angular_error()

        if self.distance > 0.01:
            linear_x = self.distance * self.kp_linear
            self.twist.linear.x = min(linear_x, self.max_vel)
        else:
            self.robot_state = 'stop'

    def stop(self):
        """
        Para o robô, publicando um comando de velocidade zero.
        """
        self.twist = Twist()

    def control(self):
        """
        Controla o robô baseado no estado atual da máquina de estados.
        Publica comandos de velocidade continuamente.
        """
        self.twist = Twist()

        if self._goal_handle is not None:
            self.state_machine[self.robot_state]()
            self.cmd_vel_pub.publish(self.twist)


def main(args=None):
    """
    Função principal que inicializa o ROS2, cria o Action Server
    e mantém o nó rodando até ser encerrado.
    """
    rclpy.init(args=args)
    action_server = GoToActionServer()
    rclpy.spin(action_server)
    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
