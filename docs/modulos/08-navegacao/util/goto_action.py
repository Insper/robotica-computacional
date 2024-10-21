import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import CancelResponse, GoalResponse
from geometry_msgs.msg import Twist, Point
from my_package.odom import Odom
from robcomp_interfaces.action import GoToPoint  # Importe sua ação

import numpy as np
import time

class GoToActionServer(Node, Odom):
    def __init__(self):
        Node.__init__(self, 'goto_action_server')
        Odom.__init__(self)

        self._action_server = ActionServer(
            self,
            GoToPoint,
            'goto_point',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback)

        self._goal_handle = None
        # self._feedback_msg = GoToPoint.Feedback()
        self._result_msg = GoToPoint.Result()

        self.twist = Twist()
        self.threshold = np.pi / 180
        self.kp_linear = 0.8
        self.kp_angular = 0.5
        self.max_vel = 0.5

        self.robot_state = 'center'
        self.state_machine = {
            'center': self.center,
            'goto': self.goto,
            'stop': self.stop
        }

        self.timer = self.create_timer(0.25, self.control)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        self._goal_handle = goal_handle
        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        self._goal_handle = None
        goal_handle.abort()
        self.cmd_vel_pub.publish(Twist())
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self.point = goal_handle.request.goal

        while rclpy.ok():
            rclpy.spin_once(self)
            if self.robot_state == 'stop':
                break

        self._result_msg.success = True
        goal_handle.succeed()

        return self._result_msg

    def get_angular_error(self):
        x = self.point.x - self.x
        y = self.point.y - self.y
        theta = np.arctan2(y, x)

        self.distance = np.sqrt(x ** 2 + y ** 2)
        erro = theta - self.yaw
        self.erro = np.arctan2(np.sin(erro), np.cos(erro))

        self.twist.angular.z = self.erro * self.kp_angular

    def center(self):
        self.get_angular_error()

        if abs(self.erro) < np.deg2rad(3):
            self.robot_state = 'goto'

    def goto(self):
        self.get_angular_error()

        if self.distance > 0.01:
            linear_x = self.distance * self.kp_linear
            self.twist.linear.x = min(linear_x, self.max_vel)
        else:
            self.robot_state = 'stop'

    def stop(self):
        self.twist = Twist()

    def control(self):
        self.twist = Twist()

        if self._goal_handle is not None:
            self.state_machine[self.robot_state]()
            self.cmd_vel_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    action_server = GoToActionServer()
    rclpy.spin(action_server)
    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""
ros2 run my_package goto_action
ros2 action send_goal /goto_point robcomp_interfaces/action/GoToPoint "{goal: {x: -3.0, y: 0.0, z: 0.0}}"
"""

