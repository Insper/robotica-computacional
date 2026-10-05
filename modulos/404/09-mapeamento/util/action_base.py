import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import CancelResponse, GoalResponse
from geometry_msgs.msg import Twist

class BaseActionServer(Node):
    def __init__(self, node_name, action_type, action_name):
        """
        Inicializa o Action Server, cria publicadores e timers necessários.
        """
        super().__init__(node_name)
        self._action_server = ActionServer(
            self,
            action_type,
            action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback
        )
        self._goal_handle = None
        self._result_msg = action_type.Result()

        # Publicador para enviar comandos ao robô (no caso de twist, mas pode ser customizado)
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
        self.cmd_vel_pub.publish(Twist())  # Publica um Twist vazio para parar o robô
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """
        Função base que deve ser sobrescrita para implementar a lógica de execução
        do objetivo específico. 
        """
        raise NotImplementedError("Subclasse deve sobrescrever execute_callback")
    
    def stop_robot(self):
        """
        Para o robô, publicando um comando de velocidade zero.
        """
        self.cmd_vel_pub.publish(Twist())
