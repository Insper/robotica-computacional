import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from rclpy.qos import ReliabilityPolicy, QoSProfile

class ImageToolNode(Node): # Mude o nome da classe

    def __init__(self, image_tuner):
        super().__init__('image_tool_node')
        self.timer = self.create_timer(0.25, self.control)

        # Subscribers
        ## Coloque aqui os subscribers
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        # Publishers
        ## Coloque aqui os publishers

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Faça aqui o processamento da imagem

    def control(self):
        print('running...')
    
def main(args=None):
    rclpy.init(args=args)
    ros_node = ImageToolNode() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()