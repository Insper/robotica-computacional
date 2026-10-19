import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from robcomp_interfaces.msg import Conversation
from robcomp_util.odom import Odom
from robcomp_util.laser import Laser
import numpy as np

class ExploradorBloqueado(Node, Odom, Laser): # Mude o nome da classe

    def __init__(self):
        super().__init__('explorador_node') # Mude o nome do nó
        Odom.__init__(self)
        Laser.__init__(self)
        self.timer = self.create_timer(0.25, self.control)

        self.sequencia = {
            'up': [-90, 90, 90, -90, -90, 180, 90, 90],
            'down': [90, -90, -90, 90, 90, 180, -90, -90],
        }
        self.estados = ['frente', 'girar', 'frente_lado', 'girar', 'andar_dead_rec', 'girar', 'frente_lado', 'girar', 'andar_dead_rec', 'girar', 'frente', 'girar', 'frente_lado', 'girar', 'frente', 'girar', 'frente', 'stop']
        self.robot_state = 'start'
        self.state_machine = {
            'stop': self.stop,
            'start': self.start,
            'girar': self.girar,
            'frente': self.frente,
            'frente_lado': self.frente_lado,
            'andar_dead_rec': self.andar_dead_rec,
        }

        # Inicialização de variáveis
        self.twist = Twist()
        
        # Subscribers
        self.handler_sub = self.create_subscription(
            Conversation,
            '/handler',
            self.handler_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        # Publishers
        self.handler_pub = self.create_publisher(Conversation, 'handler', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.history = ['nada']
        self.go = ''
        self.indice_estados = 0
        self.indice_rotacoes = 0

    def handler_callback(self, robcomp_eh_legal):
        msg = robcomp_eh_legal.message

        if msg[0] == 'H':
            print(robcomp_eh_legal)
            self.history = robcomp_eh_legal.history
            self.history.append(msg)

            ## Pegar a orientacao do handler e dar inicio
            if "cima" in msg:
                self.go = 'up'
            elif "baixo" in msg:
                self.go = 'down'
            elif "rapidamente" in msg:
                pass

    def girar(self):
        erro = self.goal_yaw - self.yaw
        erro = np.arctan2(np.sin(erro), np.cos(erro)) 
        print('ERRO: ', erro)
        if abs(erro) < np.deg2rad(2):
            self.twist.angular.z = 0.0
            self.robot_state = self.estados[self.indice_estados]
            self.indice_estados+= 1
        else:
            if erro > 0:
                self.twist.angular.z = 0.2
            else:
                self.twist.angular.z = -0.2

    def andar_dead_rec(self):
        if self.start_time is None:
            self.start_time = self.get_clock().now().to_msg().sec
        delta_tempo = self.get_clock().now().to_msg().sec - self.start_time
        tempolimite = 5

        if delta_tempo < tempolimite:
            self.twist.linear.x = 0.1
        else:
            self.twist.linear.x = 0.0
            self.robot_state = 'girar'
            self.goal_yaw = self.yaw + np.pi / 2 
            self.start_time = None  

    def frente(self):  
        self.twist.linear.x = 0.2
        front = min(self.front)
        if front < 0.5:
            self.robot_state = self.estados[self.indice_estados]
            self.indice_estados+= 1
            self.goal_yaw = self.yaw + np.deg2rad(self.sequencia[self.go][self.indice_rotacoes])
            self.indice_rotacoes += 1

    def frente_lado(self):  
        self.twist.linear.x = 0.2
        left = min(self.left)
        right = min(self.right)
        if left > 10 and right > 10:
            self.robot_state = self.estados[self.indice_estados]
            self.indice_estados+= 1
            self.goal_yaw = self.yaw + np.deg2rad(self.sequencia[self.go][self.indice_rotacoes])
            self.indice_rotacoes += 1


    def start(self):
        msg = Conversation()
        msg.message = 'Robo: Estou pronto para explorar'
        msg.history = self.history
        self.handler_pub.publish(msg)
        self.robot_state = 'stop'


    def stop(self):
        self.twist = Twist()
        if self.go != '':
            self.robot_state = self.estados[self.indice_estados]
            self.indice_estados+= 1
    
    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()

        self.cmd_vel_pub.publish(self.twist)
        
            
def main(args=None):
    rclpy.init(args=args)
    ros_node = ExploradorBloqueado() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()