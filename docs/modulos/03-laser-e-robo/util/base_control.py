import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
# Adicione aqui os imports necessários

class BaseControlNode(Node): # Mude o nome da classe

    def __init__(self):
        super().__init__('base_control_node') # Mude o nome do nó
        self.timer = self.create_timer(0.25, self.control)

        self.robot_state = 'stop'
        self.state_machine = {
            'stop': self.stop
        }

        # Inicialização de variáveis
        self.twist = Twist()
        
        # Subscribers
        ## Coloque aqui os subscribers

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        ## Coloque aqui os publishers

    def stop(self):
        self.twist = Twist()
    
    def check_danger(self):
        ## Implemente aqui a lógica de verificação de obstáculos
        # Mude o valor de self.robot_state de acordo
        pass

    def control(self):
        self.twist = Twist()
        self.check_danger()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()

        self.cmd_vel_pub.publish(self.twist)
        
            
def main(args=None):
    rclpy.init(args=args)
    ros_node = BaseControlNode() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()