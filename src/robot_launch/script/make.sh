#!/bin/bash

# ************************** 配置项 **************************
# 工作空间目录
WORKSPACE_DIR=~/XProbot
# 待编译的包列表（按原顺序整理，便于维护）
PKG_LIST=(
    robot_state_msgs
    robot_task_msgs
    cmd_vel_mux
    uuid_msgs
    geographic_msgs
    robot_launch
    dummy_robot
    rosauth
    robot_pose_publisher
    pose_init
    geometry2
    pointcloud_to_laserscan
    link_visual
    boxs_obstacle
    obstacle_stop
    point_cloud_merger
    laser_avoidance
    rosbridge_suite
    motion_control
    robot_control
    navigation_plan
    cartographer_ros
    tcp_com
    remote_control
    scan_convert
    amcl
    amcl_updater
    localization
    rosbridge_system
    robot_localization
    ros_canopen
    bluesea2
    hins_he_driver
    base_driver
    relay_control
    cross_control
    serial
)
# 编译线程数
THREAD_NUM=8
# ***********************************************************

# 函数：错误处理（打印错误信息并退出脚本）
error_exit() {
    # 错误时更新标题栏提示
    echo -e "\033]0;[编译失败] ROS包编译出错 \007"
    echo -e "\033[31m[ERROR] $1\033[0m"  # 红色字体显示错误
    exit 1
}

# 函数：进度显示（控制台输出+终端标题栏更新）
show_progress() {
    local current=$1
    local total=$2
    local pkg_name=$3
    local progress_percent=$(( (current * 100) / total ))
    
    # 1. 控制台蓝色字体显示进度（原有功能保留）
    echo -e "\033[34m[Progress: $current/$total ($progress_percent%)]\033[0m Compiling: $pkg_name"
    
    # 2. 更新终端标题栏（核心新增功能）
    # 格式：\033]0;标题内容\007 （ANSI终端标题转义序列，跨大多数终端兼容）
    echo -e "\033]0;[进度: $current/$total ($progress_percent%)] 正在编译：$pkg_name \007"
}

# 步骤1：进入工作空间目录
echo "进入工作空间目录：$WORKSPACE_DIR"
cd "$WORKSPACE_DIR" || error_exit "无法进入工作空间目录 $WORKSPACE_DIR，请检查目录是否存在"

# 步骤2：创建install目录（已存在则跳过，避免报错）
echo "创建install目录（已存在则忽略）"
mkdir -p install || error_exit "创建install目录失败"

# 步骤3：开启脚本严格模式（遇到错误立即终止，捕获命令非0返回值）
set -euo pipefail
# 补充：开启命令执行追踪（可选，如需查看详细执行过程可取消注释）
# set -x

# 步骤4：批量编译包，显示进度并处理错误
total_pkgs=${#PKG_LIST[@]}
echo -e "\033[32m开始编译，共 $total_pkgs 个包，使用 $THREAD_NUM 个线程\033[0m"  # 绿色字体提示开始编译
# 初始化终端标题栏
echo -e "\033]0;[ROS编译准备就绪] 共 $total_pkgs 个包待编译 \007"

for index in "${!PKG_LIST[@]}"; do
    # 计算当前编译序号（数组索引从0开始，序号从1开始）
    current_pkg_num=$(( index + 1 ))
    current_pkg=${PKG_LIST[$index]}
    
    # 显示当前编译进度（控制台+标题栏）
    show_progress $current_pkg_num $total_pkgs $current_pkg
    
    # 执行catkin_make编译命令
    if ! catkin_make install -j$THREAD_NUM --only-pkg-with-deps "$current_pkg"; then
        error_exit "编译包 $current_pkg 失败，脚本终止"
    fi
done

# 步骤5：编译完成提示（更新标题栏+控制台输出）
echo -e "\033[32m[SUCCESS] 所有 $total_pkgs 个包编译完成！\033[0m"
echo -e "\033]0;[ROS编译完成] 共 $total_pkgs 个包全部编译成功 \007"
exit 0
