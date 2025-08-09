import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from robcomp_util.odom import Odom
from robcomp_util.laser import Laser
from robcomp_util.andar import Andar
from robcomp_util.girar import Girar
# Adicione aqui os imports necessários

class Quadrado(Node, Odom, Laser): # Mude o nome da classe

    def __init__(self):
        super().__init__('quadrado_node') # Mude o nome do nó
        Odom.__init__(self)
        Laser.__init__(self)
        self.timer = self.create_timer(0.1, self.control)

        self.andar_node = Andar()
        self.girar_node = Girar()

        self.robot_state = 'andar'
        self.state_machine = {
            'andar': self.andar,
            'girar': self.girar
        }

        # Inicialização de variáveis
        self.twist = Twist()
        
        self.girar_node.reset()
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
    

    def andar(self):
        self.andar_node.reset()

        while not self.andar_node.robot_state == 'stop':
            rclpy.spin_once(self.andar_node)
        
        # Após andar, mudar o estado para 'girar'
        self.robot_state = 'girar'

    def girar(self):
        self.girar_node.reset()

        while not self.girar_node.robot_state == 'stop':
            rclpy.spin_once(self.girar_node)

        # Após girar, mudar o estado para 'andar'
        self.robot_state = 'andar'

    def control(self):
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()     
        self.cmd_vel_pub.publish(self.twist)
 
def main(args=None):
    rclpy.init(args=args)
    ros_node = Quadrado()

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()