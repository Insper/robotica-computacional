import time
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
from my_package.atividade3 import DistanceEstimator
import cv2
import json

class BaseNode(Node, DistanceEstimator): # Mude o nome da classe

    def __init__(self):
        Node.__init__(self, 'base_node') # Mude o nome do nó
        DistanceEstimator.__init__(self)
        self.bridge = CvBridge()
        self.subcomp = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.ranked_arucos = {}
        time.sleep(3)
        self.timer = self.create_timer(0.25, self.control)

        # Inicialização de variáveis
        
        # Subscribers
        ## Coloque aqui os subscribers

        # Publishers
        self.creeper_pub = self.create_publisher(String, 'creeper', 10)
                                                 

    def image_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        bgr, self.ranked_arucos = self.run(cv_image)

        # convert tvec to tuple
        for i, creeper in enumerate(self.ranked_arucos):
            for key in creeper:
                try:
                    self.ranked_arucos[i][key] = tuple(creeper[key].tolist())
                except:
                    pass
        print(self.ranked_arucos)

        # cv2.imshow("Imagem", bgr)
        # cv2.waitKey(1)


    def control(self):
        msg = String()
        msg.data = json.dumps(self.ranked_arucos)
        self.creeper_pub.publish(msg)
        
            
def main(args=None):
    rclpy.init(args=args)
    ros_node = BaseNode() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()