import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist

class BaseActionClientNode(Node):
    def __init__(self, node_name, action_type, action_name):
        """
        Inicializa o cliente de ação e os publishers.
        """
        super().__init__(node_name)

        # Inicializa variáveis de controle
        self.twist = Twist()
        self._goal_handle = None
        self._goal_done = False

        # Publicador para enviar comandos ao robô (Twist, por padrão)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Cliente de ação
        self._action_client = ActionClient(self, action_type, action_name)

    def send_goal(self, goal_msg):
        """
        Envia um objetivo para o Action Server e define callbacks para gerenciar o retorno.
        """
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Servidor de ação não disponível após aguardar.')
            self.stop_robot()
            return

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Callback para processar a resposta do servidor de ação sobre a aceitação do objetivo.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Objetivo rejeitado')
            self.stop_robot()
            return

        self.get_logger().info('Objetivo aceito')
        self._goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Callback para receber feedback contínuo do servidor de ação.
        Deve ser sobrescrito na classe específica.
        """
        raise NotImplementedError("Subclasse deve sobrescrever feedback_callback")

    def get_result_callback(self, future):
        """
        Callback para receber o resultado final do servidor de ação.
        """
        result = future.result().result
        self.get_logger().info(f'Resultado: {result.success}')
        self._goal_done = True

    def stop_robot(self):
        """
        Para o robô, publicando um comando de velocidade zero.
        """
        self.cmd_vel_pub.publish(Twist())
        
