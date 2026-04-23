###
# task in load parameters robot_name, mjcf_name, scene_name, node_name
# build up the temp mjcf file containing robot and scene
# launch the node
###
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

### utils
def load_mappings(config_path):
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

def find_file_recursive(root_dir, filename):
    for dirpath, dirnames, filenames in os.walk(root_dir):
        if filename in filenames:
            return os.path.join(dirpath, filename)
    return None

def create_temp_mjcf(context):
    robot_type = LaunchConfiguration('robot_type').perform(context)
    mjcf_name = LaunchConfiguration('mjcf_name').perform(context)
    scene_type = LaunchConfiguration('scene_type').perform(context)

    # load robot mjcf path
    robot_mjcf_path = os.path.join(
        get_package_share_path(robot_type+'_description'),
        'mjcf',
        mjcf_name + '.xml',
    )

    # load scene mjcf
    if scene_type != "none":
        default_scene_description_path = os.path.join(
            get_package_share_path('scene_description'),
            'mjcf',
        )
        custom_scene_description_path = os.path.join(
            get_package_share_path(robot_type+'_description'),
            'mjcf',
        )
        default_scene_mjcf_path = find_file_recursive(default_scene_description_path, scene_type + '.xml')
        custom_scene_mjcf_path = find_file_recursive(custom_scene_description_path, scene_type + '.xml')
        if default_scene_mjcf_path is not None:
            scene_mjcf_path = default_scene_mjcf_path
        elif custom_scene_mjcf_path is not None:
            scene_mjcf_path = custom_scene_mjcf_path
        else:
            raise RuntimeError(f"Cannot find scene mjcf file for scene_type: {scene_type}")
        mjcf_content = f"""
        <mujoco model="{robot_type} {scene_type}">
            <include file="{robot_mjcf_path}"/>
            <include file="{scene_mjcf_path}"/>
        </mujoco>"""

        temp_file = tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False)
        temp_file.write(mjcf_content)
        temp_file.flush() # Ensure all content is written to disk
        temp_file.close()
        print(f"Generated temporary MJCF file: {temp_file.name}")
        return [SetLaunchConfiguration('temp_mjcf_path', temp_file.name)]
    else:
        mjcf_content = f"""
        <mujoco model="{robot_type} {scene_type}">
            <include file="{robot_mjcf_path}"/>
        </mujoco>"""

        temp_file = tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False)
        temp_file.write(mjcf_content)
        temp_file.flush() # Ensure all content is written to disk
        temp_file.close()
        print(f"Generated temporary MJCF file: {temp_file.name}")
        return [SetLaunchConfiguration('temp_mjcf_path', temp_file.name)]

def delete_temp_mjcf(context):
    temp_mjcf_path = LaunchConfiguration('temp_mjcf_path').perform(context)
    if temp_mjcf_path and os.path.exists(temp_mjcf_path):
        os.remove(temp_mjcf_path)
        print(f"[MJCF TEMP] Deleted: {temp_mjcf_path}")

def generate_launch_description():
    # Declare the launch arguments
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='pendulum',
        description='Specify the robot to simulate'
    )
    mjcf_name_arg = DeclareLaunchArgument(
        'mjcf_name',
        default_value='pendulum',
        description='Specify the mjcf model name to load from the robot description package'
    )
    scene_type_arg = DeclareLaunchArgument(
        'scene_type',
        default_value='plane',
        description='''Specify the scene where the robot is simulated in,
            the model will be searched in default scene package and specified robot description package'''
    )
    node_class_name_arg = DeclareLaunchArgument(
        'node_class_name',
        default_value='PendulumSimNode',
        description='Name of the simulation node to invoke'
    )
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value='none',
        description='configuration file path for simulation node'
    )

    create_temp_mjcf_action = OpaqueFunction(function=create_temp_mjcf)

    simulation_node = Node(
        package='mujoco_sim',
        executable='simulation',
        arguments=[
            LaunchConfiguration('temp_mjcf_path'),
            LaunchConfiguration('node_class_name'),
            LaunchConfiguration('config_path')
        ],
        parameters=[LaunchConfiguration('config_path')],
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
        mjcf_name_arg,
        scene_type_arg,
        node_class_name_arg,
        config_path_arg,
        create_temp_mjcf_action,
        simulation_node,
        delete_file_handler
    ])
