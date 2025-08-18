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

def create_temp_mjcf(context): 
    model_file_name = "mjcf/go2.xml"
    model_file = os.path.join(get_package_share_path("go2_description"), model_file_name)
    terrain_file_name = "mjcf/flat_ground.xml"
    terrain_file = os.path.join(get_package_share_path("terrain_description"), terrain_file_name)

    mjcf_content = f"""
    <mujoco model="go2 flat terrain">
        <include file="{model_file}"/>
        <include file="{terrain_file}"/>
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

def generate_launch_description():

    create_temp_mjcf_action = OpaqueFunction(function=create_temp_mjcf)

    simulation_node = Node(
            package='mujoco_sim',
            executable='simulation',
            arguments=[LaunchConfiguration('temp_mjcf_path')],  # if no mjcf file provided, GUI will prompt user to select one
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
        create_temp_mjcf_action, # Execute this first to create the file
        simulation_node,
        delete_file_handler
    ])
    
