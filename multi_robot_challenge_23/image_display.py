
import rclpy
from rclpy.node import Node

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageDisplay(Node):
    def __init__(self):
        super().__init__("image_display")

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/tb3_0/camera/image_raw', self.image_callback, 10)


    def image_callback(self, data):
        # This converts the image from ROS Image message to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        cv2.imshow("Tb3 view", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageDisplay()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()