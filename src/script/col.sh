#!/bin/bash
# 兼容dash/sh：移除pipefail，改用bash兼容的错误处理
set -eu

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 获取CPU核心数（默认使用全部核心）
get_cpu_cores() {
    if command -v nproc >/dev/null 2>&1; then
        nproc
    elif command -v sysctl >/dev/null 2>&1; then
        sysctl -n hw.ncpu
    else
        echo 4  # 兜底默认值
    fi
}
DEFAULT_CORES=$(get_cpu_cores)

# 打印极简帮助（核心命令）
print_mini_help() {
    echo -e "${GREEN}Colcon 快捷工具 (CPU控制版)${NC}"
    echo "核心用法："
    echo "  $0                # 交互式菜单（默认）"
    echo "  $0 b              # 编译当前目录（使用全部CPU核心）"
    echo "  $0 b <包名>       # 编译当前目录指定包（全部核心）"
    echo "  $0 c              # 清理当前目录工作空间"
    echo "  $0 t              # 测试当前目录工作空间"
    echo "  $0 n <路径>       # 创建新工作空间（默认 ~/new_ros2_ws）"
    echo "  $0 l              # 列出当前目录包"
    echo -e "${YELLOW}进阶参数：${NC}"
    echo "  -w <路径>  指定工作空间（替代当前目录）"
    echo "  -s         符号链接安装（编译时）"
    echo "  -p <包名>  单独编译指定包（编译时）"
    echo "  -j <数字>  指定编译CPU核心数（默认：$DEFAULT_CORES核）"
    echo "  -m <参数>  传递cmake参数（编译时）"
    echo "示例："
    echo "  $0 b -j 4 -s -p my_package    # 4核编译当前目录的my_package（符号链接）"
    echo "  $0 b -w ~/ws -j 8             # 8核编译~/ws下所有包"
}

# 检查colcon安装
check_colcon() {
    if ! command -v colcon >/dev/null 2>&1; then
        echo -e "${RED}错误：未安装colcon！执行：pip install -U colcon-common-extensions${NC}"
        exit 1
    fi
}

# 交互式主菜单（极简版）
interactive_menu() {
    clear
    echo -e "\n${GREEN}===== Colcon 快捷工具 =====${NC}"
    echo -e "${YELLOW}当前工作目录：$(pwd) | 系统CPU核心数：$DEFAULT_CORES${NC}"
    echo "1. 编译工作空间（全部包）"
    echo "2. 编译指定包"
    echo "3. 清理工作空间"
    echo "4. 运行测试"
    echo "5. 创建新工作空间"
    echo "6. 列出所有包"
    echo "0. 退出"
    echo -n "${BLUE}请选择 [0-6]: ${NC}"
    read -r choice || exit 1

    case "$choice" in
        1) 
            read -r -p "是否使用符号链接安装？(y/n，默认n): " sym || sym="n"
            read -r -p "指定编译核心数（回车使用全部 $DEFAULT_CORES 核）: " cores || cores="$DEFAULT_CORES"
            cores=${cores:-$DEFAULT_CORES}
            local args=("-j" "$cores")
            [[ "$sym" == "y" ]] && args+=("-s")
            cli_build "${args[@]}"
            echo -e "${GREEN}编译完成！按回车返回菜单...${NC}"
            read -r || true; interactive_menu ;;
        2)
            read -r -p "输入要编译的包名：" pkg || exit 1
            read -r -p "是否使用符号链接安装？(y/n，默认n): " sym || sym="n"
            read -r -p "指定编译核心数（回车使用全部 $DEFAULT_CORES 核）: " cores || cores="$DEFAULT_CORES"
            cores=${cores:-$DEFAULT_CORES}
            local args=("-p" "$pkg" "-j" "$cores")
            [[ "$sym" == "y" ]] && args+=("-s")
            cli_build "${args[@]}"
            echo -e "${GREEN}编译完成！按回车返回菜单...${NC}"
            read -r || true; interactive_menu ;;
        3) cli_clean; echo -e "${GREEN}清理完成！按回车返回菜单...${NC}"; read -r || true; interactive_menu ;;
        4) cli_test; echo -e "${GREEN}测试完成！按回车返回菜单...${NC}"; read -r || true; interactive_menu ;;
        5) 
            read -r -p "输入新工作空间路径（默认 ~/new_ros2_ws）: " ws || ws="$HOME/new_ros2_ws"
            cli_create_ws "$ws"
            echo -e "${GREEN}创建完成！按回车返回菜单...${NC}"; read -r || true; interactive_menu ;;
        6) cli_list; echo -e "${BLUE}按回车返回菜单...${NC}"; read -r || true; interactive_menu ;;
        0) echo -e "${GREEN}退出！${NC}"; exit 0 ;;
        *) echo -e "${RED}无效选项！${NC}"; sleep 1; interactive_menu ;;
    esac
}

# 编译核心逻辑（支持CPU核心数、单独编译包、默认当前目录）
cli_build() {
    local ws_path="."
    local target_pkg=""
    local symlink=false
    local cmake_args=()
    local cpu_cores="$DEFAULT_CORES"  # 默认全部核心

    # 解析参数
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -w) ws_path="$2"; shift 2 ;;
            -s) symlink=true; shift ;;
            -p) target_pkg="$2"; shift 2 ;;
            -j) 
                # 校验核心数是否为有效数字
                if [[ "$2" =~ ^[0-9]+$ && "$2" -gt 0 ]]; then
                    cpu_cores="$2"
                else
                    echo -e "${YELLOW}警告：无效的核心数 $2，使用默认 $DEFAULT_CORES 核${NC}"
                fi
                shift 2 ;;
            -m) cmake_args+=("--cmake-args" "${@:2}"); break ;;
            *) echo -e "${YELLOW}警告：未知参数 $1，忽略${NC}"; shift ;;
        esac
    done

    # 检查工作空间
    if [[ ! -d "$ws_path" ]]; then
        echo -e "${RED}错误：工作空间 $ws_path 不存在${NC}"
        exit 1
    fi

    # 构建编译命令（核心：--parallel-workers 指定CPU核心数）
    local cmd=("colcon" "build" "--parallel-workers" "$cpu_cores")
    if $symlink; then
        cmd+=("--symlink-install")
    fi
    if [[ -n "$target_pkg" ]]; then
        cmd+=("--packages-select" "$target_pkg")
    fi
    if [[ ${#cmake_args[@]} -gt 0 ]]; then
        cmd+=("${cmake_args[@]}")
    fi

    # 执行编译
    echo -e "${GREEN}开始编译：${NC}"
    echo -e "  工作空间：$ws_path"
    [[ -n "$target_pkg" ]] && echo -e "  目标包：$target_pkg"
    echo -e "  CPU核心数：$cpu_cores (系统总核心：$DEFAULT_CORES)"
    cd "$ws_path" || exit 1
    "${cmd[@]}"
    
    # 提示信息
    echo -e "${GREEN}编译完成！${NC}"
    echo -e "${YELLOW}提示：source $ws_path/install/setup.bash${NC}"
}

# 清理逻辑（默认当前目录）
cli_clean() {
    local ws_path="."
    while [[ $# -gt 0 ]]; do
        case "$1" in -w) ws_path="$2"; shift 2 ;; *) shift ;; esac
    done

    if [[ ! -d "$ws_path" ]]; then
        echo -e "${RED}错误：工作空间 $ws_path 不存在${NC}"
        exit 1
    fi

    read -r -p "确认清理 $ws_path 所有编译产物？(y/n): " confirm || confirm="n"
    if [[ "$confirm" != "y" ]]; then
        echo -e "${YELLOW}取消清理${NC}"
        return 0
    fi

    echo -e "${GREEN}清理工作空间：$ws_path${NC}"
    cd "$ws_path" || exit 1
    colcon clean --all
    echo -e "${GREEN}清理完成！${NC}"
}

# 测试逻辑（默认当前目录）
cli_test() {
    local ws_path="."
    while [[ $# -gt 0 ]]; do
        case "$1" in -w) ws_path="$2"; shift 2 ;; *) shift ;; esac
    done

    if [[ ! -d "$ws_path" ]]; then
        echo -e "${RED}错误：工作空间 $ws_path 不存在${NC}"
        exit 1
    fi

    echo -e "${GREEN}运行测试：$ws_path${NC}"
    cd "$ws_path" || exit 1
    colcon test
    colcon test-result --verbose
    echo -e "${GREEN}测试完成！${NC}"
}

# 创建工作空间
cli_create_ws() {
    local ws_path="${1:-$HOME/new_ros2_ws}"
    if [[ -d "$ws_path" ]]; then
        echo -e "${YELLOW}警告：$ws_path 已存在，跳过创建${NC}"
        return 0
    fi

    echo -e "${GREEN}创建工作空间：$ws_path${NC}"
    mkdir -p "$ws_path/src" || exit 1
    cd "$ws_path" || exit 1
    # 创建时也使用CPU核心控制
    colcon build --parallel-workers "$DEFAULT_CORES" --cmake-args -DCMAKE_BUILD_TYPE=Release
    echo -e "${GREEN}创建完成！源码目录：$ws_path/src${NC}"
}

# 列出包（默认当前目录）
cli_list() {
    local ws_path="."
    while [[ $# -gt 0 ]]; do
        case "$1" in -w) ws_path="$2"; shift 2 ;; *) shift ;; esac
    done

    if [[ ! -d "$ws_path" ]]; then
        echo -e "${RED}错误：工作空间 $ws_path 不存在${NC}"
        exit 1
    fi

    echo -e "${GREEN}工作空间 $ws_path 中的包：${NC}"
    cd "$ws_path" || exit 1
    colcon list
}

# 主逻辑
main() {
    check_colcon

    # 无参数 → 交互式
    if [[ $# -eq 0 ]]; then
        interactive_menu
        exit 0
    fi

    # 极简命令解析
    case "$1" in
        b) 
            shift
            # 兼容：$0 b <包名> 这种极简写法
            if [[ $# -gt 0 && ! "$1" =~ ^- ]]; then
                cli_build -p "$1" "${@:2}"
            else
                cli_build "$@"
            fi
            ;;
        c) shift; cli_clean "$@" ;;
        t) shift; cli_test "$@" ;;
        n) shift; cli_create_ws "$@" ;;
        l) shift; cli_list "$@" ;;
        h|--help) print_mini_help ;;
        *) echo -e "${RED}未知命令：$1${NC}"; print_mini_help; exit 1 ;;
    esac
}

# 执行主逻辑
main "$@"
