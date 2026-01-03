from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.actions import LogInfo
from launch.actions import IncludeLaunchDescription  
from launch.launch_description_sources import PythonLaunchDescriptionSource  

def generate_launch_description():
    declare_map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('dummy_robot'),
            'maps',
            'map.yaml'
        ]),
        description='Path to the map yaml file'
    )

    declare_map_frame_arg = DeclareLaunchArgument(
        'map_frame_id',
        default_value='map',
        description='map frame ID'
    )

    declare_odom_frame_arg = DeclareLaunchArgument(
        'odom_frame_id',
        default_value='odom',
        description='Odometry frame ID'
    )
    
    declare_base_link_frame_arg = DeclareLaunchArgument(
        'base_link_frame_id',
        default_value='base_link',
        description='Base link frame ID'
    )

    include_dummy_robot = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('dummy_robot'),  
                'launch',                         
                'dummy_robot.launch.py'           
            ])
        )
    )

    include_cmd_vel_mux = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('cmd_vel_mux'),  
                'launch',                         
                'cmd_vel_mux.launch.py'           
            ])
        )
    )

    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', LaunchConfiguration('map_frame_id'),
            '--child-frame-id', LaunchConfiguration('odom_frame_id')
        ]
    )

    map_server_node = Node(
        package='nav2_map_server',  
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map_path'),
            'frame_id': LaunchConfiguration('map_frame_id')  
        }],
        # 标记为生命周期节点
        emulate_tty=True
    )

    robot_pose_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robot_pose_publisher'),  
                'launch',                         
                'robot_pose.launch.py'           
            ])
        )
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            # 需要管理的生命周期节点名称列表
            'node_names': ['map_server'],
            # 自动启动的状态转换顺序：configure → activate
            'autostart': True,
            # 超时时间（秒）
            'timeout': 10.0
        }]
    )

    include_navigation = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('robot_launch'), 
                'launch', 
                'navigation_start.launch.py'
            ])
        )
    )


    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # 指定自定义RViz配置文件（注释掉则用默认配置）
         arguments=['-d', PathJoinSubstitution([FindPackageShare('robot_launch'), 'rviz', 'nav.rviz'])]
    )

    # 组装launch描述
    ld = LaunchDescription()
    
    # 添加参数声明
    ld.add_action(declare_map_frame_arg)
    ld.add_action(declare_map_path_arg)
    ld.add_action(declare_odom_frame_arg)
    ld.add_action(declare_base_link_frame_arg)
    
    # 添加日志信息
    ld.add_action(LogInfo(msg="Starting dummy robot with lifecycle-managed map server!"))
    
    # 添加节点
    ld.add_action(static_tf_publisher)    
    ld.add_action(map_server_node)        
    ld.add_action(lifecycle_manager_node) 
    ld.add_action(include_dummy_robot)       
    ld.add_action(robot_pose_node)     
    ld.add_action(include_cmd_vel_mux)   
    ld.add_action(include_navigation) 
    ld.add_action(rviz2_node)       
    
    return ld