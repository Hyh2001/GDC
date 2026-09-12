"""Launch the generic recorder with its joystick adapter."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create the combined joystick recorder launch description."""
    default_config = PathJoinSubstitution(
        [FindPackageShare('recorder'), 'config', 'joy_recorder.yaml']
    )
    config = LaunchConfiguration('config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Recorder and joystick adapter parameter file',
        ),
        Node(
            package='recorder',
            executable='recorder_node',
            name='recorder',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='recorder',
            executable='joy_record_trigger',
            name='joy_record_trigger',
            output='screen',
            parameters=[config],
        ),
    ])
