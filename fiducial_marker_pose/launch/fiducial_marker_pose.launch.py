import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = os.path.join(get_package_share_path('fiducial_marker_pose'))
    config_path = os.path.join(pkg_share, 'config', 'fiducial_marker_pose.yaml')

    config_arg = DeclareLaunchArgument(name='config', default_value=config_path,
                                       description='Absolute path to config file')

    marker_estimator_node = Node(
        package='fiducial_marker_pose',
        executable='marker_estimator_node',
        namespace='camera',
        name='marker_estimator',
        output='screen',
        parameters=[LaunchConfiguration('config')]
    )

    return LaunchDescription([
        config_arg,
        marker_estimator_node
    ])
