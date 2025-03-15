import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
import numpy as np
from my_package.odom import Odom
from my_package.laser import Laser

# Adicione aqui os imports necessários

class Explorador(Node, Odom, Laser): # Mude o nome da classe

    def __init__(self):
        Node.__init__(self, 'explorador_node') # Mude o nome do nó
        Odom.__init__(self) # Mude o nome do nó
        Laser.__init__(self) # Mude o nome do nó

        self.timer = self.create_timer(0.1, self.control)
        self.openning = 15

        self.robot_state = 'girar'
        self.state_machine = {
            'stop': self.stop,
            'girar': self.girar,
            'andar':self.andar,
            'girar_horario':self.girar_horario,
            'andar_do_quadrado': self.andar_do_quadrado,
        }

        # Inicialização de variáveis
        self.twist = Twist()
        
        # Subscribers
        ## Coloque aqui os subscribers

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        ## Coloque aqui os publishers

        self.goal_girar = False
        self.goal_andar = False
        self.hora_de_sair = False
        self.vamos_embora = False

    def stop(self):
        self.twist = Twist()

    def girar(self):
        if not self.goal_girar:
            self.goal_girar = (self.yaw_2pi + np.pi / 2) % (2 * np.pi)
    
        self.twist.angular.z = 0.1
        
        diferenca_yaw = (self.goal_girar - self.yaw_2pi) % (2 * np.pi)
        print('diferenca_yaw: ', np.degrees(diferenca_yaw))
        if diferenca_yaw <= np.radians(2):
            self.goal_girar = False  
            self.twist.angular.z = 0.0
            self.robot_state = 'andar'

    def girar_horario(self):
        if not self.goal_girar:
            self.goal_girar = (self.yaw_2pi - np.pi / 2) % (2 * np.pi)
            self.hora_de_sair = True
    
        self.twist.angular.z = -0.1
        
        diferenca_yaw = (self.goal_girar - self.yaw_2pi) % (2 * np.pi)
        print('diferenca_yaw: ', np.degrees(diferenca_yaw))
        if diferenca_yaw <= np.radians(2):
            self.goal_girar = False  
            self.twist.angular.z = 0.0
            self.robot_state = 'andar'
    
    def andar(self):
        self.twist.linear.x = 0.5
        print(np.min(self.laser_msg))

        if np.min(self.front) < 0.5:
            self.robot_state = 'girar_horario'
        elif self.vamos_embora:
            self.robot_state = 'andar_do_quadrado'
        elif np.min(self.left) == np.inf and self.hora_de_sair:
            self.vamos_embora = True
            self.robot_state = 'girar'

    def andar_do_quadrado(self):
        if not self.goal_andar:
            # get time current time in seconds
            self.goal_andar = self.get_clock().now().to_msg().sec

        self.twist.linear.x = 0.5

        # time elapsed in seconds
        time_elapsed = self.get_clock().now().to_msg().sec - self.goal_andar
        print('time_elapsed: ', time_elapsed)
        if time_elapsed >= 5:
            self.goal_andar = False
            self.twist.linear.x = 0.0
            self.robot_state = 'stop'

    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()

        self.cmd_vel_pub.publish(self.twist)
        
            
def main(args=None):
    rclpy.init(args=args)
    ros_node = Explorador() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()