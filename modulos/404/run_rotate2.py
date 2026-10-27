import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from my_package.odom import Odom
from my_package.rotate2 import Rotate2 # Pode colocar em outra lugar

class Caller(Node,Odom):
    def __init__(self):
        Node.__init__(self,'rot_node')
        Odom.__init__(self)
        time.sleep(1)
        self.timer = self.create_timer(0.2, self.control)
        self.angles = [0,180, 30] # Sequencia de angulos que o robô deve visitar
        self.i = 0
        self.rotate_node = Rotate2(ang=self.angles[self.i], deg=True)

    def control(self):
        if not self.rotate_node.robot_state == 'para':
            print('Rotating')
            rclpy.spin_once(self.rotate_node) # Roda o controle do rotate_node uma vez
        else:
            if self.i == len(self.angles):
                print('Finished')
                return
            print(f'Rotated goal {self.angles[self.i]}')
            self.rotate_node.robot_state = 'gira' # Reseta o estado do rotate_node
            self.rotate_node.get_goal_from_target(self.angles[self.i], deg=True) # Atualiza o angulo desejado
            self.i += 1

def main(args=None):
    rclpy.init(args=args)
    ros_node = Caller()

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()