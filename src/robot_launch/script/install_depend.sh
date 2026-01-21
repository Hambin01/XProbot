#!/bin/bash
set -eu

home_dir="/home/b1"
# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# 基础提示函数
info() { echo -e "\n${GREEN}[INFO]${NC} $1"; }
warn() { echo -e "\n${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "\n${RED}[ERROR]${NC} $1"; exit 1; }
sep() { echo -e "\n=============================================\n"; }

# 软链接检查函数
check_link() {
    [ -L "$1" ] && [ "$(readlink -f "$1")" = "$(readlink -f "$2")" ] && return 0
    return 1
}

# 软链接创建函数
create_link() {
    if check_link "$1" "$2"; then
        info "$3链接已正确配置，跳过"
    else
        [ -e "$1" ] && [ ! -L "$1" ] && sudo mv "$1" "${1}_backup_$(date +%Y%m%d_%H%M%S)"
        [ -L "$1" ] && sudo rm -f "$1"
        info "配置$3链接: $1 -> $2"
        sudo ln -s "$2" "$1"
        check_link "$1" "$2" || error "$3链接配置失败"
    fi
}

# 权限检查
[ "$(id -u)" -ne 0 ] && { warn "需要root权限"; sudo -v; }
while true; do sudo -n true; sleep 60; kill -0 "$$" || exit; done 2>/dev/null &

# 主程序
sep; info "开始安装配置"; sep
cd $home_dir
# 设置目标目录
TARGET_DIR=$([ "$1" == "install" ] && echo "install/share" || echo "src")
info "目标目录: $TARGET_DIR"

# 1. 配置APT源
sep; info "1. 配置软件源"
REPO_FILE="/etc/apt/sources.list.d/autolabor.list"
REPO_CONTENT="deb [trusted=yes arch=amd64] http://deb.repo.autolabor.com.cn jammy main"
if [ -f "$REPO_FILE" ]; then
    grep -qxF "$REPO_CONTENT" "$REPO_FILE" || { warn "更新软件源"; echo "$REPO_CONTENT" | sudo tee "$REPO_FILE"; }
else
    info "添加软件源"; echo "$REPO_CONTENT" | sudo tee "$REPO_FILE"
fi
sudo apt update -y

# 2. 安装系统包
sep; info "2. 安装系统依赖"
install_pkg() {
    dpkg -s "$1" >/dev/null 2>&1 && info "$1已安装" || { info "安装$1"; sudo apt install -y "$1"; }
}
#install_pkg "ros-noetic-autolabor"
install_pkg "libsdl1.2-dev"
install_pkg "libsdl-image1.2"
install_pkg "libmuparser-dev"
install_pkg "libgtest-dev"
install_pkg "libgmock-dev"
install_pkg "nginx"
install_pkg "ros-humble-cartographer"

# 3. 安装Python包
sep; info "3. 安装Python依赖"
install_pip() {
    python3 -m pip show "$1" >/dev/null 2>&1 && info "$1已安装" || { info "安装$1"; sudo pip3 install "$1"; }
}
install_pip "twisted"
install_pip "pyopenssl"
install_pip "autobahn"
install_pip "tornado"
install_pip "pymongo"
install_pip "service_identity"

# 4. 编译安装orocos-kdl
sep; info "4. 安装orocos-kdl库"
OROCOS_LIB="/usr/local/lib/liborocos-kdl.so.1.5.4"
OROCOS_LINK="/usr/local/lib/liborocos-kdl.so.1.5.1"

if [ -f "$OROCOS_LIB" ] && check_link "$OROCOS_LINK" "$OROCOS_LIB"; then
    info "orocos-kdl已安装"
else
    info "编译安装orocos-kdl"
    rm -rf orocos_kinematics_dynamics
    git clone https://github.com/orocos/orocos_kinematics_dynamics.git || error "克隆仓库失败"
    
    cd orocos_kinematics_dynamics/orocos_kdl || exit
    mkdir -p build && cd build || exit
    command -v cmake || sudo apt install -y cmake build-essential
    cmake .. && make -j$(nproc) && sudo make install
    
    create_link "$OROCOS_LINK" "$OROCOS_LIB" "orocos-kdl"
    sudo ldconfig
    
    cd ~ && rm -rf orocos_kinematics_dynamics
    [ -f "$OROCOS_LIB" ] && check_link "$OROCOS_LINK" "$OROCOS_LIB" || error "orocos-kdl安装失败"
    info "orocos-kdl安装完成"
fi

# 5. 配置Cartographer
sep; info "5. 配置Cartographer"
CARTO_DEST="/opt/ros/humble/share/cartographer/configuration_files"
CARTO_SRC="$home_dir/XProbot/$TARGET_DIR/robot_launch/carto_file"
create_link "$CARTO_DEST" "$CARTO_SRC" "Cartographer配置"
sudo chmod 777 -R "$CARTO_SRC"

# 6. 配置Nginx
sep; info "6. 配置Nginx"
ROBOT_VIEW_SRC="$home_dir/XProbot/$TARGET_DIR/robot_launch/script/robot_view"
ROBOT_VIEW_DEST="/var/www/"
ROBOT_CONF_SRC="$home_dir/XProbot/$TARGET_DIR/robot_launch/script/robot.conf"
ROBOT_CONF_DEST="/etc/nginx/conf.d/robot.conf"

sudo cp -r  "$ROBOT_VIEW_SRC" "$ROBOT_VIEW_DEST"
create_link "$ROBOT_CONF_DEST" "$ROBOT_CONF_SRC" "Nginx配置"

sudo chmod -R 777 "$ROBOT_VIEW_DEST/robot_view"


# 7. 创建SSL证书
sep; info "7. 配置SSL证书"
SSL_DIR="/etc/nginx/ssl"
sudo mkdir  "$SSL_DIR"
if [ -f "$SSL_DIR/hambin.key" ] && [ -f "$SSL_DIR/hambin.crt" ]; then
    info "SSL证书已存在"
else
    info "生成SSL证书"
    cd "$SSL_DIR" || exit
    sudo openssl genrsa -out hambin.key 2048 >/dev/null 2>&1
    sudo openssl req -x509 -new -nodes -key hambin.key -sha256 -days 3650 -out hambin.crt \
        -subj "/C=CN/ST=Guangdong/L=Shenzhen/O=Internal/OU=IT/CN=hambin.com" >/dev/null 2>&1
    [ -f "$SSL_DIR/hambin.key" ] && [ -f "$SSL_DIR/hambin.crt" ] || error "SSL证书生成失败"
fi
sudo systemctl restart nginx.service

sep; info "所有配置完成！"
echo -e "=============================================\n"
