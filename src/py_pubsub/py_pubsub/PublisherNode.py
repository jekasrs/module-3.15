import rclpy
import cv2
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header 

class PublisherNode(Node):
    def __init__(self, capture):
        super().__init__('invader_node')
        self.publisher_ = self.create_publisher(Image, 'sensors_msgs/image', 10)
        self.capture = capture 
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.capture.isOpened():
            
            self.get_logger().info("[INFO]: start of publishing an image")
            ret, frame = self.capture.read()
            
            msg = Image()
            header = Header()
            header.frame_id = 'cur_frame'

            img_bytes = np.array(frame.flatten(), dtype=np.uint8)

            msg.header = header
            msg.height = frame.shape[0]
            msg.width  = frame.shape[1]
            msg.encoding = "bgr8"
            msg.is_bigendian = False
            msg.step = np.shape(frame)[2] * np.shape(frame)[1]
            msg.data = img_bytes.tolist()
            
            self.publisher_.publish(msg)
            self.get_logger().info('[INFO]: end of publishing an image')


def main(args=None):
    capture = cv2.VideoCapture(0)

    rclpy.init(args=args)
    print("[INFO]: invader_node init successful")

    node = PublisherNode(capture)
    rclpy.spin(node)
    node.destroy_node()

    print("[INFO]: invader_node shutdown successful")
    rclpy.shutdown()

    cv2.destroyAllWindows()
    capture.release()


if __name__ == '__main__':
    main()
