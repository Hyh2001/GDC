from launch import LaunchDescription
from launch.actions import SetLaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit

import tempfile
import yaml 

def load_mappings(config_path):
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

def create_temp_mjcf(context): 
    config_path = os.path.join(
        get_package_share_path('mujoco_sim'),
        'config',
        'mjcf_mappings.yaml'
    )
    mappings = load_mappings(config_path)
    
    robot_type = LaunchConfiguration('robot_type').perform(context)
    scene_type = LaunchConfiguration('scene_type').perform(context)

    robot_map = mappings['robots']
    scene_map = mappings['scenes']

    if robot_type not in robot_map:
        raise RuntimeError(f"Unknown robot_type: {robot_type}")
    if scene_type not in scene_map:
        raise RuntimeError(f"Unknown terrain_type: {scene_type}")

    robot_pkg = robot_map[robot_type]['package'] 
    robot_model_file = robot_map[robot_type]['model']   
    scene_pkg = scene_map[scene_type]['package']
    scene_model_file = scene_map[scene_type]['model']

    model_file = os.path.join(get_package_share_path(robot_pkg), robot_model_file)
    scene_file = os.path.join(get_package_share_path(scene_pkg), scene_model_file)

    mjcf_content = f"""
    <mujoco model="{robot_type} {scene_type}">
        <include file="{model_file}"/>
        <include file="{scene_file}"/>
    </mujoco>"""

    temp_file = tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False)
    temp_file.write(mjcf_content)
    temp_file.flush() # Ensure all content is written to disk
    temp_file.close()

    print(f"Generated temporary MJCF file: {temp_file.name}")
    return [SetLaunchConfiguration('temp_mjcf_path', temp_file.name)]

def delete_temp_mjcf(context, *args, **kwargs): 
    temp_mjcf_path = LaunchConfiguration('temp_mjcf_path').perform(context)
    if temp_mjcf_path and os.path.exists(temp_mjcf_path):
        os.remove(temp_mjcf_path)
        print(f"[MJCF TEMP] Deleted: {temp_mjcf_path}")

def get_node_class(context): 
    config_path = os.path.join(
        get_package_share_path('mujoco_sim'),
        'config',
        'mjcf_mappings.yaml'
    )
    mappings = load_mappings(config_path)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    ground_truth = LaunchConfiguration('ground_truth').perform(context).lower() == 'true'
    robot_map = mappings['robots']
    if robot_type not in robot_map:
        raise RuntimeError(f"Unknown robot_type: {robot_type}")
    if ground_truth == True:
        node_name = robot_map[robot_type]['nodes']['ground_truth']
    elif ground_truth == False:
        node_name = robot_map[robot_type]['nodes']['normal']
    else:
        raise RuntimeError(f"Invalid ground_truth value: {ground_truth}")
    if node_name == "NotImplemented":
        raise RuntimeError(f"Node class for robot_type '{robot_type}' with ground_truth='{ground_truth}' is not implemented.")
    return [SetLaunchConfiguration('node_name', node_name)]

def generate_launch_description():

    # Declare launch arguments for robot_type and ground_truth
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='go2',
        description='Specify the robot to simulate'
    )
    scene_type_arg = DeclareLaunchArgument(
        'scene_type',
        default_value='flat_ground',
        description='Specify the scene where the robot is simulated in'
    )
    ground_truth_arg = DeclareLaunchArgument(
        'ground_truth',
        default_value='false',
        description='Include ground truth data (true/false)'
    )

    create_temp_mjcf_action = OpaqueFunction(function=create_temp_mjcf)
    get_node_class_action = OpaqueFunction(function=get_node_class)

    simulation_node = Node(
        package='mujoco_sim',
        executable='simulation',
        arguments=[
            LaunchConfiguration('temp_mjcf_path'),
            LaunchConfiguration('node_name')
        ],
        name='simulation',
        output='screen'
    )

    delete_file_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=simulation_node,
            on_exit=[
                OpaqueFunction(function=delete_temp_mjcf)
            ]
        )
    )

    return LaunchDescription([
        robot_type_arg,
        scene_type_arg,
        ground_truth_arg,
        create_temp_mjcf_action,
        get_node_class_action,
        simulation_node,
        delete_file_handler
    ])
    
