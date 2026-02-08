from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [FindPackageShare('uav_perception'), 'config', 'yolo_detector.yaml']
    )

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to YAML params file for uav_perception yolo_detector',
    )

    node = Node(
        package='uav_perception',
        executable='yolo_detector',
        name='yolo_detector',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([config_arg, node])
