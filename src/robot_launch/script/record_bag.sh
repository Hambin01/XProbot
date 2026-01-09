#!/bin/bash

# ===================== 配置区 =====================
# 默认录制的bag文件保存路径（不含后缀）
DEFAULT_BAG_NAME="/home/robot/record_bag"
# 默认录制的话题（如果用户未指定）
DEFAULT_TOPICS=("/laser" )
# ===================== 配置区结束 =====================

# 全局变量
bag_name=""
topics=()

# 打印脚本使用说明
function show_help() {
    echo "Usage: $0 [command] [options]"
    echo "Commands:"
    echo "  record    Start recording rosbag (default topics: ${DEFAULT_TOPICS[*]})"
    echo "  end       Stop recording rosbag"
    echo "Options for record command:"
    echo "  -t <topics>  Specify topics to record (space-separated, e.g., '-t /imu /tf')"
    echo "  -o <path>    Specify output bag file path (without .bag suffix)"
    echo "Examples:"
    echo "  $0 record                    # Record default topics to default path"
    echo "  $0 record -t /imu /tf        # Record /imu and /tf to default path"
    echo "  $0 record -t /rslidar_points -o /tmp/my_bag  # Record specified topic to custom path"
    echo "  $0 end                       # Stop recording"
}

# 检查话题是否存在
function check_topic_exists() {
    local topic="$1"
    if ! rostopic info "$topic" >/dev/null 2>&1; then
        echo "Error: Topic $topic is not available! Please check the topic and try again."
        exit 1
    fi
}

# 启动录制
function start_record() {
    # 设置默认值
    if [ ${#topics[@]} -eq 0 ]; then
        topics=("${DEFAULT_TOPICS[@]}")
    fi
    if [ -z "$bag_name" ]; then
        bag_name="$DEFAULT_BAG_NAME"
    fi

    echo "======================================"
    echo "Starting rosbag recording..."
    echo "Output file: $bag_name.bag"
    echo "Topics to record: ${topics[*]}"
    echo "======================================"

    # 检查所有指定的话题是否存在
    for topic in "${topics[@]}"; do
        check_topic_exists "$topic"
    done

    # 删除已存在的同名bag文件（避免覆盖提示）
    if [ -f "$bag_name.bag" ]; then
        echo "Removing existing bag file: $bag_name.bag"
        rm -f "$bag_name.bag"
    fi

    # 启动录制（后台运行）
    rosbag record "${topics[@]}" -O "$bag_name.bag" &
    record_pid=$!
    echo "Recording started with PID: $record_pid"
}

# 停止录制并修复active文件
function stop_record() {
    echo "======================================"
    echo "Stopping rosbag recording..."
    echo "======================================"

    # 查找rosbag record进程
    pids=$(pgrep -f "rosbag record")
    if [ -n "$pids" ]; then
        echo "Terminating rosbag record processes: $pids"
        # 使用sudo终止进程（保留你的原逻辑，建议实际使用时确认是否需要sudo）
        echo 'nvidia' | sudo -S kill -9 $pids
        # 等待进程退出
        sleep 1
    else
        echo "No rosbag record processes found"
    fi

    # 修复active bag文件（如果存在）
    active_bag="$bag_name.bag.active"
    if [ -z "$bag_name" ]; then
        active_bag="$DEFAULT_BAG_NAME.bag.active"
    fi
    
    if [ -f "$active_bag" ]; then
        echo "Detected active bag file: $active_bag"
        echo "Performing repair operation..."
        # 重新索引并修复
        rosbag reindex "$active_bag"
        rosbag fix "$active_bag" "$bag_name.bag"
        # 清理临时文件
        rm -f "$bag_name.bag.orig.active"
        rm -f "$active_bag"
        echo "Repair operation complete. Final bag file: $bag_name.bag"
    else
        echo "No active bag file detected."
    fi
}

# ===================== 主逻辑 =====================
# 检查是否安装了ROS环境
if ! command -v rostopic &>/dev/null; then
    echo "Error: ROS environment is not setup! Please source ROS setup.bash first."
    exit 1
fi

# 解析命令行参数
if [ $# -eq 0 ]; then
    show_help
    exit 1
fi

case "$1" in
    record)
        # 跳过第一个参数（record），解析后续参数
        shift
        while [ $# -gt 0 ]; do
            case "$1" in
                -t)
                    shift
                    # 读取所有后续的话题参数，直到遇到下一个选项
                    while [ $# -gt 0 ] && [[ "$1" != -* ]]; do
                        topics+=("$1")
                        shift
                    done
                    ;;
                -o)
                    shift
                    bag_name="$1"
                    shift
                    ;;
                *)
                    echo "Error: Unknown option $1"
                    show_help
                    exit 1
                    ;;
            esac
        done
        start_record
        ;;
    end)
        # 如果指定了输出路径，也传递给stop函数
        shift
        while [ $# -gt 0 ]; do
            case "$1" in
                -o)
                    shift
                    bag_name="$1"
                    shift
                    ;;
                *)
                    echo "Error: Unknown option $1"
                    show_help
                    exit 1
                    ;;
            esac
        done
        stop_record
        ;;
    *)
        echo "Error: Invalid command $1"
        show_help
        exit 1
        ;;
esac

exit 0
