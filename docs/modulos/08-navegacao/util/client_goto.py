from geometry_msgs.msg import Point
from robcomp_interfaces.action import GoToPoint  # Importe a ação
import rclpy
from geometry_msgs.msg import Twist

class GoToActionClient(BaseActionClientNode):
    """
    Cliente de ação específico para enviar objetivos de movimentação para um ponto.
    Herda da classe BaseActionClientNode e implementa a lógica específica do GoTo.
    Inclui a máquina de estados e controle.
    """

    def __init__(self):
        """
        Inicializa o cliente de ação específico de 'GoToPoint'.
        """
        super().__init__('goto_action_client', GoToPoint, 'goto_point')

        # Estado inicial da máquina de estados
        self.robot_state = 'goto'
        self.state_machine = {
            'goto': self.goto,
            'waiting_for_result': self.waiting_for_result,
            'stop': self.stop
        }

        # Cria um timer para controlar o robô
        self.timer = self.create_timer(0.25, self.control)

    def goto(self):
        """
        Envia um objetivo de movimentação para um ponto específico.
        """
        goal_msg = GoToPoint.Goal()
        goal_msg.goal = Point(x=-3.0, y=0.0, z=0.0)
        self.send_goal(goal_msg)
        self.robot_state = 'waiting_for_result'

    def waiting_for_result(self):
        """
        Verifica se o objetivo foi concluído e muda para o estado 'stop' se terminado.
        """
        if self._goal_done:
            self.robot_state = 'stop'

    def stop(self):
        """
        Para o robô, publicando um comando de velocidade zero.
        """
        self.twist = Twist()

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
