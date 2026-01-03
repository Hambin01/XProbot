from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # 获取包路径
    motion_control_package = FindPackageShare('motion_control')
    config_file_path = PathJoinSubstitution([
        motion_control_package,
        'config',
        'motion_control_indoor.yaml'
    ])

    # 定义运动控制节点
    motion_control_node = Node(
        package='motion_control',          # 包名（与ROS 1一致）
        executable='motion_control_node',  # 可执行文件名称
        name='motion_control_node',        # 节点名称（与ROS 1一致）
        output='screen',                   # 输出到屏幕（对应ROS 1的output="screen"）
        respawn=True,                      # 自动重启（对应ROS 1的respawn="true"）
        parameters=[
            # 加载参数文件（对应ROS 1的rosparam load）
            config_file_path,
            # 可选：直接设置额外参数
            {'clear_params': True}
        ],
        # 命名空间（对应ROS 1的ns="/motion_control_node"）
        namespace='/motion_control_node',
        # 额外参数
        arguments=['--ros-args', '--log-level', 'info'],
        # 确保节点关闭时清理参数（对应ROS 1的clear_params="true"）
        on_exit=Node(
            package='rclpy_executables',
            executable='param_delete',
            arguments=['--namespace', '/motion_control_node', '--all'],
            output='screen'
        )
    )

    # 构建启动描述
    ld = LaunchDescription()
    ld.add_action(motion_control_node)
    
    return ld