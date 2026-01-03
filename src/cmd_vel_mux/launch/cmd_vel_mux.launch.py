import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('cmd_vel_mux')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    print(f"Params file path: {params_file}")

    return LaunchDescription([
        Node(
            package='cmd_vel_mux',
            executable='cmd_vel_mux_node',
            name='cmd_vel_mux_node',
            output='screen',
            respawn=True,
            parameters=[
                {
                    'yaml_cfg_file': params_file 
                }
            ],
            arguments=['--log-level', 'DEBUG']
        ),
        Node(
            package='cmd_vel_mux',
            executable='heart_cmd_node',
            name='heart_cmd_vel_node',
            output='screen',
            respawn=True
        )
    ])
