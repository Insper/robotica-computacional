import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from rclpy.qos import ReliabilityPolicy, QoSProfile
from main_window import ImageTuner
import sys
from PyQt5.QtWidgets import QApplication

class ImageToolNode(Node):
    def __init__(self, image_tuner):
        super().__init__('image_tool_node')
        self.image_tuner = image_tuner
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.image_tuner.image_update_signal.signal.emit(cv_image)

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    image_tuner = ImageTuner()
    image_tuner.show()

    ros_node = ImageToolNode(image_tuner=image_tuner)

    # Running ROS spin in a separate thread
    import threading
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    exit_code = app.exec_()
    rclpy.shutdown()  # Ensure ROS is shutdown properly
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
