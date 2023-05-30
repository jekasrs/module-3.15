from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    pkg_share = get_package_share_path('fiducial_marker_pose')
    default_config_params_path = pkg_share / 'config/params.yaml'
    
    declare_marker_estimator_params_yaml = DeclareLaunchArgument(
        name="marker_estimator_params_yaml",
        default_value = str(default_config_params_path),
        description = "Путь до файла config/params.yaml"
    )

    node = Node(
            package = 'fiducial_marker_pose',
            executable = 'marker_estimator',
            name = 'marker_estimator',
            output = 'screen',
            parameters = [
                {"camera_driver_params_yaml": LaunchConfiguration("marker_estimator_params_yaml")}
            ]
        )
    
    return LaunchDescription([
        declare_marker_estimator_params_yaml,
        node
    ])
