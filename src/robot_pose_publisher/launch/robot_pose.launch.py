from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    args = [
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('publish_frequency', default_value='10.0'),
        DeclareLaunchArgument('is_stamped', default_value='true'),
        # 节点配置
        Node(
            package='robot_pose_publisher',
            executable='robot_pose_publisher',
            name='robot_pose_publisher_node',
            output='screen',
            respawn=True,
            parameters=[{
                'map_frame': LaunchConfiguration('map_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'publish_frequency': LaunchConfiguration('publish_frequency'),
                'is_stamped': LaunchConfiguration('is_stamped')
            }]
        )
    ]
    return LaunchDescription(args)