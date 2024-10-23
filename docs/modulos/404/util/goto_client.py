import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, Point
from robcomp_interfaces.action import GoToPoint  # Importe a ação

class BaseControlNode(Node):

    def __init__(self):
        super().__init__('base_control_node')
        self.timer = self.create_timer(0.25, self.control)

        self.robot_state = 'goto'
        self.state_machine = {
            'goto': self.goto,
            'waiting_for_result': self.waiting_for_result,
            'stop': self.stop
        }

        # Inicialização de variáveis
        self.twist = Twist()
        
        # Subscribers
        ## Coloque aqui os subscribers

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Action client
        self._action_client = ActionClient(self, GoToPoint, 'goto_point')
        self._goal_handle = None
        self._goal_done = False

    def goto(self):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available after waiting')
            self.robot_state = 'stop'
            return

        goal_msg = GoToPoint.Goal()
        goal_msg.goal = Point(x=-3.0, y=0.0, z=0.0)

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.robot_state = 'waiting_for_result'

    def waiting_for_result(self):
        if self._goal_done:
            self.robot_state = 'stop'

    def stop(self):
        self.twist = Twist()

    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()
    
    def move_robot(self):
        self.cmd_vel_pub.publish(self.twist)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.status}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.robot_state = 'stop'
            return

        self.get_logger().info('Goal accepted')
        self._goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}')
        self._goal_done = True

def main(args=None):
    rclpy.init(args=args)
    ros_node = BaseControlNode()

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
