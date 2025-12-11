from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution,Command
from launch_ros.substitutions import FindPackageShare 
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessStart, OnProcessExit

import os 

def generate_launch_description():
    # Define the robot_description from xacro
    urdf_file = PathJoinSubstitution([
        FindPackageShare("tron1_description"),
        "urdf",
        "tron1_flat_foot.urdf"
    ])

    # Load ros2_control config YAML
    # config_file = os.path.join(
    #     get_package_share_directory("pendulum_description"),
    #     "config",
    #     "robot_control.yaml"
    # )
    
    mujoco_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("mujoco_sim"),
            "launch",
            "sim_launch.py"
        )),
        launch_arguments={
            'robot_type': 'tron1',
            'mjcf_name': 'tron1_flat_foot',
            'scene_type': 'flat_ground',
            'node_class_name': 'Tron1SimNode',
            'config_path': '/home/yuhao/ros2_ws/deploy_ws/src/deployment_code_base/communication/interfaces/test/tron1_sim_config.yaml'  # no ros2_control for tron1 for now
        }.items()
    )
    
    # Start robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['cat ', urdf_file])}],
        output='screen',
        # remappings=[
        #     ('/robot_description', '/controller_manager/robot_description')
        # ]
    )

    # Start ros2_control_node
    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[config_file],
    #     output="screen"
    # )
    

    return LaunchDescription([
        mujoco_sim_launch,
        # ros2_control_node,
        # robot_state_publisher_node,

        # load_pendulum_dummy_estimator,
        # load_pendulum_periodic_planner,
        # load_pendulum_pid_controller,
        # load_pendulum_velocity_policy_controller,
        # RegisterEventHandler( 
        #     event_handler=OnProcessExit(
        #         target_action=load_pendulum_dummy_estimator, # load_pendulum_pid_controller
        #         on_exit=[load_pendulum_pid_controller]                 
        #     )
        # ),
        # RegisterEventHandler( 
        #     event_handler=OnProcessExit(
        #         target_action=load_pendulum_dummy_estimator, # load_pendulum_pid_controller
        #         on_exit=[load_pendulum_velocity_policy_controller]                 
        #     )
        # ),
        # pendulum_pid_controller_spawner,
        # pendulum_dummy_estimator_spawner,
        # joint_state_broadcaster_spawner
    ])
