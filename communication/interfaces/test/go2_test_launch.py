from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution,Command
from launch_ros.substitutions import FindPackageShare 
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit

import os 

def generate_launch_description():
    # Define the robot_description from xacro
    urdf_file = PathJoinSubstitution([
        FindPackageShare("go2_description"),
        "urdf",
        "go2.urdf"
    ])

    # Load ros2_control config YAML
    # config_file = os.path.join(
    #     get_package_share_directory("interfaces"),
    #     "test",
    #     "go2_config.yaml"
    # )
    config_file = os.path.join(
        get_package_share_directory("go2_description"),
        "config",
        "robot_control.yaml"
    )
    
    mujoco_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("mujoco_sim"),
            "launch",
            "sim_launch.py"
        )),
        launch_arguments={
            'robot_type': 'go2',
            'scene_type': 'flat_ground',
            'ground_truth': 'true'
        }.items()
    )
    
    # Start robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['cat ', urdf_file])}],
        output='screen',
        remappings=[
            ('/robot_description', '/controller_manager/robot_description')
        ]
    )

    # Start ros2_control_node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[config_file],
        output="screen"
    )
    
    load_go2_dummy_estimator = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "go2_dummy_estimator",
        ],
        output="screen",
    )
    
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )


    return LaunchDescription([
        mujoco_sim_launch,
        ros2_control_node,
        robot_state_publisher_node,
        load_joint_state_broadcaster,
        load_go2_dummy_estimator,
        # joint_state_broadcaster_spawner
    ])
