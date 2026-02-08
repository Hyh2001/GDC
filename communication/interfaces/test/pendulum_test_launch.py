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
        FindPackageShare("pendulum_description"),
        "urdf",
        "pendulum.urdf"
    ])

    # Load ros2_control config YAML
    config_file = os.path.join(
        get_package_share_directory("pendulum_description"),
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
            'robot_type': 'pendulum',
            'mjcf_name': 'pendulum',
            'scene_type': 'flat_ground',
            'node_class_name': 'PendulumSimNode'
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
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[config_file],
        output="screen"
    )

    pendulum_dummy_estimator_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pendulum_dummy_estimator", "--controller-manager", "/controller_manager"],
    )

    pendulum_pid_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pendulum_pid_controller", "--controller-manager", "/controller_manager"],
    )

    # Load joint state broadcaster
    load_pendulum_dummy_estimator = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'active',
            'pendulum_dummy_estimator'
        ],
        output='screen'
    )

    load_pendulum_periodic_planner = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'active',
            'pendulum_joint_periodic_planner'
        ],
        output='screen'
    )

    # Load joint trajectory controller
    load_pendulum_pid_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'inactive',
            'pendulum_pid_controller'
        ],
        output='screen'
    )

    load_pendulum_velocity_policy_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'active',
            'pendulum_velocity_policy_controller'
        ],
        output='screen'
    )


    return LaunchDescription([
        mujoco_sim_launch,
        ros2_control_node,
        robot_state_publisher_node,
        # load_pendulum_pid_controller,
        # RegisterEventHandler(
        #     event_handler=OnExecutionComplete(
        #         target_action=robot_state_publisher_node,
        #         on_completion=[load_pendulum_pid_controller]
        #     )
        # ),
        # TimerAction(
        #     period=3.0, # wait for hardware interface and simulation to ready
        #     actions=[load_pendulum_pid_controller]
        # ),
        load_pendulum_dummy_estimator,
        load_pendulum_periodic_planner,
        # load_pendulum_pid_controller,
        # load_pendulum_velocity_policy_controller,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_pendulum_dummy_estimator, # load_pendulum_pid_controller
                on_exit=[load_pendulum_pid_controller]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_pendulum_dummy_estimator, # load_pendulum_pid_controller
                on_exit=[load_pendulum_velocity_policy_controller]
            )
        ),
        # pendulum_pid_controller_spawner,
        # pendulum_dummy_estimator_spawner,
        # joint_state_broadcaster_spawner
    ])
