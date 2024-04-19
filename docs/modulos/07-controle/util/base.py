import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
# Adicione aqui os imports necessários

class BaseNode(Node): # Mude o nome da classe

    def __init__(self):
        super().__init__('base_node') # Mude o nome do nó
        self.timer = self.create_timer(0.25, self.control)

        # Inicialização de variáveis
        
        # Subscribers
        ## Coloque aqui os subscribers

        # Publishers
        ## Coloque aqui os publishers

    def control(self):
        print('running...')
        
            
def main(args=None):
    rclpy.init(args=args)
    ros_node = BaseNode() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()