import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np


class PublisherNode(Node):
    def __init__(self):
        super().__init__('imager')
        self.publisher_ = self.create_publisher(Image, 'image', 10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cv_image = cv2.imread('send.png')

    def timer_callback(self):
        self.publisher_.publish(self.bridge.cv2_to_imgmsg(np.array(self.cv_image), "bgr8"))
        self.get_logger().info('Publishing an image')


def main(args=None):
    rclpy.init(args=args)

    node = PublisherNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()