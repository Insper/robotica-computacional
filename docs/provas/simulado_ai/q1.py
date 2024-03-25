import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
# Adicione aqui os imports necessários
from my_package.odom import Odom
from my_package.laser import Laser
import numpy as np

class Explorador(Node, Odom, Laser): # Mude o nome da classe

    def __init__(self):
        Node.__init__(self, 'explorador_node')
        Odom.__init__(self)
        Laser.__init__(self)
        self.timer = self.create_timer(0.25, self.control)
        self.counter = 0

        self.robot_state = 'girar_anti_horario'
        self.state_machine = {
            'stop': self.stop,
            'girar_anti_horario': self.girar_anti_horario,
            'girar_horario': self.girar_horario,
            'andar': self.andar,
            'andar_final': self.andar_final,
        }

        # Inicialização de variáveis
        self.twist = Twist()
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.goal_girar = False
        self.goal_andar = False
        self.wz = 0.2
        self.final = False
    
    def girar_anti_horario(self):
        if not self.goal_girar:
            self.goal_girar = (self.yaw_2pi + np.pi / 2) % (2 * np.pi)
    
        self.twist.angular.z = self.wz
        
        diferenca_yaw = (self.goal_girar - self.yaw_2pi) % (2 * np.pi)
        print('diferenca_yaw: ', np.degrees(diferenca_yaw))
        if diferenca_yaw <= np.radians(5):
            if not self.final:
                self.goal_girar = False  
                self.twist.angular.z = 0.0
                self.robot_state = 'andar'
                self.final = True
            else:
                self.robot_state = 'andar_final'

    def girar_horario(self):
        if not self.goal_girar:
            self.goal_girar = (self.yaw_2pi - np.pi / 2) % (2 * np.pi)
    
        self.twist.angular.z = -self.wz
        
        diferenca_yaw = (self.yaw_2pi - self.goal_girar) % (2 * np.pi)
        print('diferenca_yaw: ', np.degrees(diferenca_yaw))
        if diferenca_yaw <= np.radians(2):
            self.goal_girar = False  
            self.twist.angular.z = 0.0
            self.robot_state = 'andar'
            self.counter += 1

    def andar(self):
        self.twist.linear.x = 0.5

        if self.counter == 4:
            self.openning = 15
            if np.min(self.left) == np.inf:
                self.twist.linear.x = 0.0
                self.robot_state = 'girar_anti_horario'
                self.counter = 0
                self.wz = 0.2

        elif np.min(self.front) < 0.9:
            self.twist.linear.x = 0.0
            self.robot_state = 'girar_horario'

    def andar_final(self):
        if not self.goal_andar:
            # get time current time in seconds
            self.goal_andar = self.get_clock().now().to_msg().sec

        self.twist.linear.x = 0.5

        # time elapsed in seconds
        time_elapsed = self.get_clock().now().to_msg().sec - self.goal_andar
        print('time_elapsed: ', time_elapsed)
        if time_elapsed >= 5:
            self.twist.linear.x = 0.0
            self.robot_state = 'stop'

    def stop(self):
        self.twist = Twist()

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