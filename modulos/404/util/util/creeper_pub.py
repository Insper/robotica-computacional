import time
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
from robcomp_util.creeper_detector import CreeperDetector
import cv2
import json
from robcomp_interfaces.msg import DetectionArray, Detection

class BaseNode(Node, CreeperDetector): # Mude o nome da classe

    def __init__(self):
        Node.__init__(self, 'creeper_detector_node') # Mude o nome do nó
        CreeperDetector.__init__(self)
        self.bridge = CvBridge()
        self.subcomp = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Publishers
        self.creeper_pub = self.create_publisher(DetectionArray, 'creeper', 10)
                                                 

    def image_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        _, self.ranked_arucos = self.run(cv_image)

        # convert tvec to tuple
        msg = DetectionArray()
        for i, creeper in enumerate(self.ranked_arucos):
            detection = Detection()
            detection.classe = str(creeper['color']) + '-' + str(creeper['id'][0])
            detection.cx = float(cv_image.shape[1]) / 2 - float(creeper['body_center'][0])
            detection.cy = 0.

            msg.deteccoes.append(detection)
        
        self.creeper_pub.publish(msg)
            
            
def main(args=None):
    rclpy.init(args=args)
    ros_node = BaseNode() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()