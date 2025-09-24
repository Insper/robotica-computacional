# IMPORTAÇÃO DE BIBLIOTECAS NECESSÁRIAS:
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist, Point
from robcomp_util.andar import Andar
from simulado_ai_99.girar import Girar
from robcomp_util.laser import Laser
from robcomp_util.odom import Odom
from robcomp_interfaces.msg import GameStatus



# --------------------------------------------------

class Jogador(Node, Laser, Odom): 

    def __init__(self):
        super().__init__('jogador_node')
        Laser.__init__(self)
        Odom.__init__(self)
        rclpy.spin_once(self)

        self.andar_node = Andar()
        self.girar_node = Girar()

        self.robot_state = 'ate_parede'
        self.temp_state = None
        self.state_machine = {
            'ate_parede':self.ate_parede,
            'ate_vitoria':self.ate_vitoria,
            'ate_y': self.ate_y,
            'girar': self.girar,
            'done': self.done,
            'esperar': self.esperar,
        }

        self.twist = Twist()
        self.young_hee_pub = self.create_publisher(GameStatus, 'young_hee', 10)
        self.young_hee_sub = self.create_subscription(
            GameStatus,
            '/young_hee',
            self.young_hee_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        game = GameStatus()
        game.status = "READY"
        game.player_name = "ROBCOMP_EH_LEGAL"
        self.young_hee_pub.publish(game)
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.started = False

        # Inicializa o timer
        self.timer = self.create_timer(0.1, self.control)
        rclpy.spin_once(self, timeout_sec=0.0)
    
    def young_hee_callback(self,msg):
        print('\n', msg.current_word, 'temp_state:', self.temp_state)
        if msg.current_word in ["2", "3"]:
            # Só salva o estado se não estiver já esperando
            if self.robot_state != "esperar":
                self.temp_state = self.robot_state
                self.robot_state = "esperar"
        elif msg.current_word in ["ba"] and self.temp_state:
            print("Voltando ao estado anterior:", self.temp_state)
            self.robot_state = self.temp_state
            self.temp_state = None  # Reseta o estado temporário após usar

    def esperar(self):
        self.twist = Twist()
    
    def ate_parede(self):
        self.twist = Twist()
        self.twist.linear.x = 0.3

        if min(self.front) < 0.6:
            self.twist = Twist()
            self.robot_state = 'girar'
            self.rotacao = -90 # Assumindo que girar recebe o ângulo em graus


    def girar(self):
        if not self.started:
            print("\nIniciando movimento de rotação...")
            rclpy.spin_once(self.girar_node)
            self.girar_node.reset(rotacao=self.rotacao)
            self.started = True

        # Foi nescessário remover o while, pois travava o callback da Young Hee
        if self.robot_state == "esperar":
            return
        rclpy.spin_once(self.girar_node)

        if self.girar_node.robot_state == 'done':
            if self.rotacao == -90:
                self.robot_state = "ate_y"
            elif self.rotacao == 90:
                self.robot_state = "ate_vitoria"
            self.started = False
    
    def ate_y(self):
        self.twist = Twist()
        self.twist.linear.x = 0.3

        print('Y:', self.y)

        if self.y > 0.0:
            self.twist = Twist()
            self.rotacao = 90 # Assumindo que girar recebe o ângulo em graus
            self.robot_state = 'girar'

    def ate_vitoria(self):
        self.twist = Twist()
        self.twist.linear.x = 0.3

        if self.x < -5.0:
            self.twist = Twist()
            self.robot_state = "done"

    def done(self):
        print("\nQuadrado percorrido com sucesso!")
        self.twist = Twist()

    def control(self):
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()
        
        if not self.robot_state in ['girar']:
            self.cmd_vel_pub.publish(self.twist)
 
def main(args=None):
    rclpy.init(args=args)  # Inicia o ROS2
    ros_node = Jogador()  # Cria o nó

    while ros_node.robot_state != 'done':  # Enquanto o robô não terminar a ação
        rclpy.spin_once(ros_node)  # Processa os callbacks e o timer

    ros_node.destroy_node()  # Destroi o nó
    rclpy.shutdown()  # Finaliza o ROS2

if __name__ == '__main__':
    main()