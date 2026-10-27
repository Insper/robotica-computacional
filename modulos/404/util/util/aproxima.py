import time
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
import numpy as np
from robcomp_interfaces.msg import DetectionArray, Detection
from robcomp_util.laser import Laser


class AproximaCreeper(Node, Laser):

    def __init__(self):
        super().__init__('aproxima_creeper_node')
        Laser.__init__(self)
        
        self.subcomp = self.create_subscription(
            DetectionArray,
            'creeper',
            self.creeper_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.target = 'blue-21'
        self.cx = np.inf

        self.timer = self.create_timer(0.25, self.control)

        self.robot_state = 'segue'
        self.state_machine = {
            'segue': self.segue,
            'stop': self.stop
        }

        self.threshold = 5
        self.kp = 0.005

        # Inicialização de variáveis
        self.twist = Twist()

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def creeper_callback(self, msg):
        for detection in msg.deteccoes:
            if detection.classe == self.target:
                self.cx = detection.cx
                break
        else:
            self.cx = np.inf
    
    def calc_erro(self):
        self.erro = self.cx
        self.rot = self.erro * self.kp
        print('Erro Angular:', self.erro)
        
    def segue(self):
        if self.cx == np.inf:
            self.twist.angular.z = -0.4
        else:
            self.calc_erro()
            print(min(self.front))
            self.twist.linear.x = min(0.2, min(self.front))
            self.twist.angular.z = self.rot

            if min(self.front) < 0.1:
                self.robot_state = 'stop'
                self.stop()
    
    def stop(self):
        self.twist = Twist()

    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()
        self.cmd_vel_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    ros_node = AproximaCreeper()

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()