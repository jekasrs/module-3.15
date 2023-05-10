import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image


class DetectionNode(Node):

    def __init__(self):
        super().__init__('Detection Node')

        video_qos = QoSProfile(depth=10, reliability=QoSProfile.ReliabilityPolicy.RELIABLE,
                               durability=QoSProfile.DurabilityPolicy.VOLATILE)

        self.subscription = self.create_subscription(Image, "image", self.imageRectifiedCallback, qos_profile=video_qos)

    def imageRectifiedCallback(self, msg):
        self.get_logger().info("Rectified image received from Package\tSize: %dx%d - Timestamp: %u.%u sec ",
                               msg.width, msg.height, msg.header.stamp.sec, msg.header.stamp.nanosec)


def main(args=None):
    rclpy.init(args=args)

    node = DetectionNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
