import cv2
import numpy as np
import math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
from .detector import Detector
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import TransformStamped
from pose_interfaces.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker as VizMarker
from visualization_msgs.msg import MarkerArray as VizMarkerArray
from builtin_interfaces.msg import Duration

class MarkerEstimator(Node):
    def __init__(self):
        super().__init__('marker_estimator')
        self.__intrinsics = None
        self.__distortion_coefficients = None
        self.__detector = None
        self.__bridge = CvBridge()
        self.__duration_1_sec = Duration(sec=1)
        self.__config = self.get_config()
        self.__aruco_type = self.__config['aruco_type']
        self.__marker_size = self.__config['marker_size']
        self.__is_compressed = self.__config['use_compressed']
        self.__is_debug = self.__config['debug']

        if self.__is_compressed:
            self.__sub_image = \
                self.create_subscription(CompressedImage, 'image_compressed',
                                         self.image_callback, qos_profile_sensor_data)
        else:
            self.__sub_image = \
                self.create_subscription(Image, 'image_raw',
                                           self.image_callback, qos_profile_sensor_data)

        self.__sub_camera_info = \
            self.create_subscription(CameraInfo, 'camera_info', self.camera_info_callback, 1)

         self.__pub_marker_pose = \
             self.create_publisher(MarkerArray, 'marker_pose', qos_profile_sensor_data)

        if self.__is_debug:
            self.__pub_image_debug = \
                self.create_publisher(Image, 'image_debug', qos_profile_sensor_data)
            self.tf_broadcaster = TransformBroadcaster(self)
            self.__pub_visualisation_rviz = \
                self.create_publisher(VizMarkerArray, 'image_rviz', qos_profile_sensor_data)

    def get_config(self):
        self.declare_parameter('aruco_type', 'DICT_4X4_1000')
        self.declare_parameter('marker_size', 0.1)
        self.declare_parameter('use_compressed', False)
        self.declare_parameter('debug', False)

        config = {
            'aruco_type': self.get_parameter('aruco_type').value,
            'marker_size': self.get_parameter('marker_size').value,
            'use_compressed': self.get_parameter('use_compressed').value,
            'debug': self.get_parameter('debug').value
        }

        return config

    def camera_info_callback(self, msg):
        if self.__detector is None:
            self.__intrinsics = np.float32(msg.k)
            self.__distortion_coefficients = np.array(msg.d)
            self.__detector = Detector(self.__aruco_type,
                                       self.__intrinsics,
                                       self.__distortion_coefficients)

    def image_callback(self, msg):
        if self.__detector is None:
            self.get_logger().warning('Detector is not initialised!')
            return

        image = None
        if self.__is_compressed:
            image = self.__bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        else:
            image = self.__bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        self.__detector.feed(image)

        if len(self.__detector.get_corners()) > 0:
            markers_msg = MarkerArray()
            markers_viz_msg = VizMarkerArray()
            axis_viz_markers = []
            markers_msg.header.frame_id = msg.header.frame_id
            markers_msg.header.stamp = self.get_clock().now().to_msg()
            for i in range(0, len(self.__detector.get_ids())):
                corners = self.__detector.get_corners()[i]
                rvec, tvec = self.__detector.get_pose(self.__marker_size, corners)

                rotation_matrix = np.eye(3)
                rotation_matrix, _ = cv2.Rodrigues(np.array(rvec))

                r = Rotation.from_matrix(rotation_matrix)
                quaternion = r.as_quat()

                # Custom Marker message
                marker = Marker()
                marker.id = int(self.__detector.get_ids()[i])

                # Store the translation information
                marker.pose.position.x = tvec[0][0]
                marker.pose.position.y = tvec[1][0]
                marker.pose.position.z = tvec[2][0]

                # Store the rotation information
                marker.pose.orientation.x = quaternion[0]
                marker.pose.orientation.y = quaternion[1]
                marker.pose.orientation.z = quaternion[2]
                marker.pose.orientation.w = quaternion[3]

                markers_msg.markers.append(marker)
                self.get_logger().info(f'Marker with id {str(marker.id)} is detected!')

                if self.__is_debug:
                    viz_marker = self.create_viz_marker(marker)
                    markers_viz_msg.markers.append(viz_marker)

                    axis_marker = self.create_viz_axis(marker)
                    axis_viz_markers.append(axis_marker)

            self.__pub_marker_pose.publish(markers_msg)

            if self.__is_debug:
                image_debug = self.__detector.visualise(self.__marker_size)
                msg_debug = self.__bridge.cv2_to_imgmsg(image_debug, encoding='bgr8')
                self.__pub_image_debug.publish(msg_debug)

                # RVIZ2
                self.__pub_visualisation_rviz.publish(markers_viz_msg)
                for xyz in axis_viz_markers:
                    self.tf_broadcaster.sendTransform(xyz)

    def create_viz_axis(self, marker):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_link'
        t.child_frame_id = str(marker.id)

        t.transform.translation.x = marker.pose.position.x
        t.transform.translation.y = marker.pose.position.y
        t.transform.translation.z = marker.pose.position.z

        t.transform.rotation.x = marker.pose.orientation.x
        t.transform.rotation.y = marker.pose.orientation.y
        t.transform.rotation.z = marker.pose.orientation.z
        t.transform.rotation.w = marker.pose.orientation.w

        return t

    def create_viz_marker(self, marker):
        marker_viz = VizMarker()

        marker_viz.id = marker.id
        marker_viz.type = marker_viz.CUBE
        marker_viz.action = marker_viz.ADD
        marker_viz.lifetime = self.__duration_1_sec
        marker_viz.header.frame_id = 'camera_link'

        marker_viz.scale.x = self.__marker_size
        marker_viz.scale.y = self.__marker_size
        marker_viz.scale.z = self.__marker_size * 0.1

        marker_viz.color.a = 1.0
        marker_viz.color.r = 1.0
        marker_viz.color.g = 1.0
        marker_viz.color.b = 0.0

        marker_viz.pose.position.x = marker.pose.position.x
        marker_viz.pose.position.y = marker.pose.position.y
        marker_viz.pose.position.z = marker.pose.position.z

        marker_viz.pose.orientation.x = marker.pose.orientation.x
        marker_viz.pose.orientation.y = marker.pose.orientation.y
        marker_viz.pose.orientation.z = marker.pose.orientation.z
        marker_viz.pose.orientation.w = marker.pose.orientation.w

        return marker_viz
