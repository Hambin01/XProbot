from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # ========== MoveBase 导航核心 ==========
    # 导航参数文件（包含代价地图、路径规划、控制器参数）
    nav2_params_file = PathJoinSubstitution(
        [FindPackageShare("robot_launch"), "config", "nav2_params.yaml"]
    )

    # 导航生命周期管理（管理AMCL、MoveBase相关节点）
    lifecycle_manager_nav_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"autostart": True},
            {"node_names": [
                "controller_server",  # 运动控制器
                "planner_server",     # 路径规划器
                "bt_navigator"        # 行为树导航器
            ]}
        ]
    )

    # 加载MoveBase核心节点（通过Nav2的launch文件批量启动）
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("robot_launch"), "launch", "navigation_launch.py"])
        ]),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": nav2_params_file,
            "map_subscribe_transient_local": "true"  # 地图话题使用瞬态本地订阅（确保节点启动后能获取地图）
        }.items()
    )


    # ========== 机器人状态发布（可选，若已单独启动则注释） ==========
    # 若机器人没有单独发布TF（底座->激光雷达、底座->里程计等），需添加以下节点
    # robot_state_publisher_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     output="screen",
    #     parameters=[
    #         {"use_sim_time": use_sim_time},
    #         {"robot_description": Command(["xacro ", PathJoinSubstitution(
    #             [FindPackageShare("your_robot_description"), "urdf", "robot.urdf.xacro"]
    #         )])}
    #     ]
    # )

    # ========== 组装Launch描述 ==========
    return LaunchDescription([
        nav2_bringup_launch,
        lifecycle_manager_nav_node,
    ])