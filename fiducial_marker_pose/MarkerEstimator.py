import cv2
import numpy as np
import rclpy
import yaml

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image

from fiducial_marker_pose.Detector import Detector
from fiducial_marker_pose.msg import Point2D
from fiducial_marker_pose.msg import Marker
from fiducial_marker_pose.msg import MarkerArray

class MarkerEstimator(Node):

    def __init__(self):
        super().__init__('marker_estimator')
        video_qos = QoSProfile(    
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.counter = 0
        self.subscription = self.create_subscription(Image, "image_raw", self.imageRectifiedCallback, qos_profile=video_qos)
        self.publisher_ = self.create_publisher(MarkerArray, 'marker_raw', 10)

        self.declare_parameter('marker_estimator_params_yaml')
        params_yaml_path = self.get_parameter('marker_estimator_params_yaml').get_parameter_value().string_value
        
        with open(params_yaml_path, 'r') as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            aruco_type = config['aruco_type']
            matrix_coefficients = np.array(config['matrix_coefficients'])
            distortion_coefficients = np.array(config['distortion_coefficients'])
            detector = Detector(aruco_type, matrix_coefficients, distortion_coefficients)
            self.detector = detector

    def imageRectifiedCallback(self, msg):
        height = msg.height
        width = msg.width
        channel = msg.step//msg.width
        frame = np.reshape(msg.data, (height, width, channel))

        self.detector.feed(frame)
        if len(self.detector.getCorners()) > 0:

            # [debug] save file
            img = self.detector.visualise()
            flipped_img = cv2.flip(img, 1)
            file_name = 'received' + str(self.i) + '.png'
            cv2.imwrite(file_name, flipped_img)
            self.get_logger().info('[DETECTION]: file has saved')

            # construct message
            markers_msg = MarkerArray()

            for i in range(0, len(self.detector.getIds())):
                self.counter += 1

                # get pose
                marker = self.detector.aruco_get_pose_of_marker(self.detector.getCorners()[i], self.detector.getIds()[i])
                markers_msg[i] = marker

                # [debug] logger
                str_info = "[DETECTION]: marker {id}= " + marker.id
                self.get_logger().info(str_info)

            # send
            self.publisher_.publish(markers_msg)


def main(args=None):

    rclpy.init(args=args)
    print("[INFO]: marker_estimator init successful")

    node = MarkerEstimator()
    rclpy.spin(node)
    node.destroy_node()

    print("[INFO]: marker_estimator shutdown successful")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
