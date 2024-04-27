import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from my_package.odom import Odom

class Rotate2(Node,Odom):

    def __init__(self, ang, deg=False):
        Node.__init__(self,'rotate2_node')
        Odom.__init__(self)
        time.sleep(1)
        self.timer = self.create_timer(0.2, self.control)

        self.robot_state = 'gira'
        self.state_machine = {
            'gira': self.gira,
            'para': self.para,
        }

        self.get_goal_from_target(ang, deg)

        self.threshold = 5
        self.kp = .5

        # Inicialização de variáveis
        self.twist = Twist()

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def get_goal_from_target(self,ang, deg):
        if deg:
            ang = np.deg2rad(ang)
        self.goal_yaw = (ang + np.pi) % (2 * np.pi) - np.pi

    def gira(self):
        erro = self.goal_yaw - self.yaw
        erro = np.arctan2(np.sin(erro), np.cos(erro))

        self.twist.angular.z = self.kp * erro

        if abs(erro) < np.deg2rad(1):
            self.robot_state = 'para'
            self.twist = Twist()
    
    def para(self):
        self.twist = Twist()       

    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()
        self.cmd_vel_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    ros_node = Rotate2(275, deg=True)

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()