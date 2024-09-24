import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
from robcomp_util.laser import Laser
from robcomp_util.odom import Odom
from geometry_msgs.msg import Twist

class Quadrado(Node, Laser, Odom):

    def __init__(self):
        super().__init__('quadrado_node')
        Laser.__init__(self)
        Odom.__init__(self)
        self.timer = self.create_timer(0.15, self.control)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.robot_state = 'anda'
        self.state_machine = {
            'anda': self.anda,
            'gira': self.gira,
            'stop': self.stop,
        }
        self.twist = Twist()
        self.goal_yaw = 0.0

        self.distancia = 2
        self.velocidade = 0.2

        current_time = self.get_clock().now().to_msg()
        current_time = float(current_time.sec) + float(current_time.nanosec)/10**9
        self.goal_time = current_time + self.distancia / self.velocidade

    def anda(self):
        current_time = self.get_clock().now().to_msg()
        current_time = float(current_time.sec) + float(current_time.nanosec)/10**9

        if current_time >= self.goal_time:
            self.goal_yaw = self.yaw + np.pi/2
            self.robot_state = "gira"
        else:
            self.twist.linear.x = self.velocidade
    
    def gira(self):
        erro = self.goal_yaw - self.yaw
        erro = np.arctan2(np.sin(erro), np.cos(erro))

        if abs(erro) < np.deg2rad(1):
            self.robot_state = "anda"
            current_time = self.get_clock().now().to_msg()
            current_time = float(current_time.sec) + float(current_time.nanosec)/10**9
            self.goal_time = current_time + self.distancia / self.velocidade
        else:
            if erro < 0:
                self.twist.angular.z = -0.2
            else:
                self.twist.angular.z = 0.2

    def stop(self):
        self.twist = Twist()

    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()
        self.vel_pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    ros_node = Quadrado()

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()