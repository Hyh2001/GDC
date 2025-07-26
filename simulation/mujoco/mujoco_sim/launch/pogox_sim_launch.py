from launch import LaunchDescription
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    # model_file_name = "model/champoline.xml"
    model_file_name = "model/PogoX_xml.xml"
    # model_file_name = "model/PogoX_Trampoline.xml"
    model_file = os.path.join(get_package_share_path("pogox_model"), model_file_name)
    # print(model_file)
    
    urdf_file_name = "model/PogoX_URDF.urdf"
    urdf_file = os.path.join(get_package_share_path("pogox_model"), urdf_file_name)
    
    config = os.path.join(
        get_package_share_path('pogox_control'),
        'config',
        'pogox_control_config.yaml'
    )
    return LaunchDescription([
        Node(
            package='mujoco_sim',
            executable='simulation',
            arguments=[model_file],  # if no mjcf file provided, GUI will prompt user to select one
            name='simulation',
            output='screen'
        )
    ])