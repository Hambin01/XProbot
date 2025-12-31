from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.actions import LogInfo

def generate_launch_description():

    declare_odom_frame_arg = DeclareLaunchArgument('odom_frame_id',default_value='odom',description='Odometry frame ID')
    declare_base_link_frame_arg = DeclareLaunchArgument('base_link_frame_id',default_value='base_link',description='Base link frame ID')

    dummy_robot_node = Node(
        package='dummy_robot',
        executable='dummy_robot_node',
        name='dummy_robot_node',
        output='screen',
        respawn=True,
        parameters=[
            {
                'odom_frame_id': LaunchConfiguration('odom_frame_id'),
                'base_link_frame_id': LaunchConfiguration('base_link_frame_id')
            }
        ],
        remappings=[
                ('/cmd_vel', '/smoother_cmd_vel'),
            ],
        arguments=['--ros-args', '--log-level', 'INFO']
    )

    # 组装launch描述
    ld = LaunchDescription()
    
    ld.add_action(declare_odom_frame_arg)
    ld.add_action(declare_base_link_frame_arg)
    
    # 添加日志信息
    ld.add_action(LogInfo(msg="Starting dummy robot with lifecycle-managed map server!"))
    
    # 添加节点
    ld.add_action(dummy_robot_node)       # 最后启动机器人节点
    
    return ld