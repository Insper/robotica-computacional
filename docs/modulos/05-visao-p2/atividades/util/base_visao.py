import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import String

class ImageToolNode(Node): # Mude o nome da classe

    def __init__(self, image_tuner):
        super().__init__('image_tool_node')
        self.runnable = True

        # Subscribers
        ## Coloque aqui os subscribers
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        self.flag_sub = self.create_subscription(
            String,
            '/vision/image_flag', # Mude o nome do tópico
            self.flag_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        # Publishers
        ## Coloque aqui os publishers

    def flag_callback(self, msg):
        self.runnable = bool(msg.data)


    def image_callback(self, msg):
        if self.runnable:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Faça aqui o processamento da imagem
        else:
            print('Image processing is paused')

    
def main(args=None):
    rclpy.init(args=args)
    ros_node = ImageToolNode() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()