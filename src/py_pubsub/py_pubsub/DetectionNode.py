import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from py_pubsub.utils import aruco_display, aruco_detect, aruco_get_pose_of_marker, draw_all_vectors

class DetectionNode(Node):

    def __init__(self):
        super().__init__('detection_node')

        video_qos = QoSProfile(    
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        
        self.i = 0
        self.subscription = self.create_subscription(Image, "sensors_msgs/image", self.imageRectifiedCallback, qos_profile=video_qos)

    def imageRectifiedCallback(self, msg):
        height = msg.height
        width = msg.width
        channel = msg.step//msg.width
        frame = np.reshape(msg.data, (height, width, channel))
        
        corners, ids = aruco_detect(frame)
        if len(corners) != 0:
            rvec_arr, tvec_arr = aruco_get_pose_of_marker(corners, ids)
            img = aruco_display(corners, ids, frame)
            img = draw_all_vectors(img, rvec_arr, tvec_arr)
            flipped_img = cv2.flip(img, 1)

            self.i += 1
            str_info = "[DETECTION]: vectors are: r=" + str(rvec_arr) + "t=" + str(tvec_arr)
            self.get_logger().info(str_info)

            file_name = 'received' + str(self.i) + '.png'
            cv2.imwrite(file_name, flipped_img)
            self.get_logger().info('[DETECTION]: file has saved')


def main(args=None):
    rclpy.init(args=args)
    print("[INFO]:detection_node init successful")

    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()

    print("[INFO]: detection_node shutdown successful")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
