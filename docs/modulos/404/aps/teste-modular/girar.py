import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from robcomp_util.odom import Odom
from robcomp_util.laser import Laser
# Adicione aqui os imports necessários

class Girar(Node, Odom, Laser): # Mude o nome da classe

    def __init__(self):
        super().__init__('girar_node') # Mude o nome do nó
        Odom.__init__(self)
        Laser.__init__(self)
        self.timer = self.create_timer(0.25, self.control)

        self.robot_state = 'andar'
        self.state_machine = {
            'andar': self.andar,
            'girar': self.girar
        }

        # Inicialização de variáveis
        self.reset()
        self.giro = 0.2
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
    
    def reset(self):
        self.twist = Twist()
        self.goal_yaw = self.yaw + np.pi / 2
    
    def girar(self):
        
        self.erro = self.goal_yaw - self.yaw 
        self.erro = np.arctan2(np.sin(self.erro), np.cos(self.erro)) 

        print(f'Erro: {np.degrees(self.erro)}')

        if self.erro > 0:
            self.twist.angular.z = self.giro
        else: 
            self.twist.angular.z = -self.giro

        if abs(self.erro) < np.radians(2):
                self.robot_state = 'stop'
    
    def stop(self):
        self.twist = Twist()
        print("Parando o robô.")

    def control(self):
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()     
        self.cmd_vel_pub.publish(self.twist)
 
def main(args=None):
    rclpy.init(args=args)
    ros_node = Girar()

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()