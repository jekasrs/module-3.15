import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image

from Detector import Detector

class MarkerEstimator(Node):

    def __init__(self):
        super().__init__('marker_estimator')
        video_qos = QoSProfile(    
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.i = 0
        self.subscription = self.create_subscription(Image, "image_raw", self.imageRectifiedCallback, qos_profile=video_qos)

        self.aruco_type = "DICT_4X4_1000"
        self.detector = Detector(aruco_type)
        self.detector.setShowMarkers(True)
        self.detector.setShowVectors(True)

    def imageRectifiedCallback(self, msg):
        height = msg.height
        width = msg.width
        channel = msg.step//msg.width
        frame = np.reshape(msg.data, (height, width, channel))

        self.detector.feed(frame)
        if len(self.detector.getCorners()) > 0:
            self.i += 1

            rvec_arr, tvec_arr = Detector.aruco_get_pose_of_marker(self.detector.getCorners(), self.detector.getIds())

            img = self.detector.visualise()
            flipped_img = cv2.flip(img, 1)
            file_name = 'received' + str(self.i) + '.png'
            cv2.imwrite(file_name, flipped_img)
            self.get_logger().info('[DETECTION]: file has saved')

            for r, t in zip(rvec_arr, tvec_arr):
                str_info = "[DETECTION]: vectors are: r=" + str(r) + " t=" + str(t)
                self.get_logger().info(str_info)


def main(args=None):
    rclpy.init(args=args)
    print("[INFO]: detection_node init successful")

    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()

    print("[INFO]: detection_node shutdown successful")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
