import rclpy
from .marker_estimator import MarkerEstimator


def main(args=None):
    rclpy.init(args=args)
    marker_estimator_node = MarkerEstimator()
    marker_estimator_node.get_logger().info('Initialization is successful!')
    rclpy.spin(marker_estimator_node)
    marker_estimator_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
