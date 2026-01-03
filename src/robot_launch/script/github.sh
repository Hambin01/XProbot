#!/bin/bash
set -euo pipefail

# 定义颜色输出（增强可读性）
RED='\033[31m'
GREEN='\033[32m'
YELLOW='\033[33m'
NC='\033[0m' # 重置颜色

# 定义不同系统的hosts路径（备份文件名去掉时间戳）
if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" ]]; then
    # Windows（Git Bash/Cygwin）
    HOSTS_PATH="/c/Windows/System32/drivers/etc/hosts"
    BACKUP_PATH="/c/Windows/System32/drivers/etc/hosts.bak"  # 去掉日期
    SUDO_CMD=""  # Windows需手动以管理员运行，无法自动sudo
elif [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS
    HOSTS_PATH="/etc/hosts"
    BACKUP_PATH="/etc/hosts.bak"  # 去掉日期
    SUDO_CMD="sudo"
else
    # Linux
    HOSTS_PATH="/etc/hosts"
    BACKUP_PATH="/etc/hosts.bak"  # 去掉日期
    SUDO_CMD="sudo"
fi

# 检查IP参数是否传入
if [ $# -ne 1 ]; then
    echo -e "${RED}错误：请传入要替换的GitHub IP地址作为参数！${NC}"
    echo -e "${YELLOW}使用示例：${NC}"
    echo -e "  Linux/macOS:  ./replace_github_hosts.sh 140.82.113.4"
    echo -e "  Windows:      bash replace_github_hosts.sh 140.82.113.4（需管理员Git Bash）"
    exit 1
fi

# 验证IP格式
GITHUB_IP="$1"
if ! [[ "$GITHUB_IP" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
    echo -e "${RED}错误：IP地址格式无效！请输入如 140.82.113.4 格式的IP${NC}"
    exit 1
fi

# 验证IP是否可达（可选，增加容错）
ping -c 1 -W 2 "$GITHUB_IP" > /dev/null 2>&1 || {
    echo -e "${YELLOW}警告：IP $GITHUB_IP 可能不可达，仍继续替换...${NC}"
}

# 备份hosts文件
echo -e "${GREEN}1/4 正在备份hosts文件到：$BACKUP_PATH${NC}"
$SUDO_CMD cp "$HOSTS_PATH" "$BACKUP_PATH" || {
    echo -e "${RED}错误：备份hosts失败！请确保有管理员/root权限${NC}"
    exit 1
}

# 步骤1：删除原有GitHub相关条目
echo -e "${GREEN}2/4 正在清理原有GitHub相关hosts条目${NC}"
# 过滤掉包含github.com的行，保留其他内容
$SUDO_CMD awk '!/github\.com/' "$HOSTS_PATH" > "$HOSTS_PATH.tmp" || {
    echo -e "${RED}错误：清理原有条目失败！${NC}"
    #exit 1
}

# 步骤2：添加新的GitHub IP条目
echo -e "${GREEN}3/4 正在添加新的GitHub IP条目：$GITHUB_IP${NC}"
cat >> "$HOSTS_PATH.tmp" << EOF

# GitHub Fast IP (auto-updated by script)
$GITHUB_IP    github.com
$GITHUB_IP    www.github.com
EOF

# 步骤3：替换原hosts文件
$SUDO_CMD mv "$HOSTS_PATH.tmp" "$HOSTS_PATH" || {
    echo -e "${RED}错误：替换hosts文件失败！${NC}"
    # 回滚备份
    $SUDO_CMD cp "$BACKUP_PATH" "$HOSTS_PATH"
    exit 1
}

# 步骤4：刷新DNS缓存（跨平台）
echo -e "${GREEN}4/4 正在刷新DNS缓存${NC}"
if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" ]]; then
    # Windows刷新DNS
    ipconfig /flushdns > /dev/null 2>&1
elif [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS刷新DNS
    sudo dscacheutil -flushcache
    sudo killall -HUP mDNSResponder > /dev/null 2>&1
else
    # Linux刷新DNS（适配不同发行版）
    if command -v systemd-resolve > /dev/null 2>&1; then
        sudo systemd-resolve --flush-caches > /dev/null 2>&1
    elif [ -f /etc/resolv.conf ]; then
        sudo systemctl restart NetworkManager > /dev/null 2>&1 || true
    fi
fi

# 输出完成信息（备份路径已更新）
echo -e "\n${GREEN}====================================${NC}"
echo -e "${GREEN}操作完成！GitHub hosts已更新为：${NC}"
echo -e "${YELLOW}$GITHUB_IP    github.com${NC}"
echo -e "${YELLOW}$GITHUB_IP    www.github.com${NC}"
echo -e "${GREEN}====================================${NC}"
echo -e "${YELLOW}提示：${NC}"
echo -e "1. 原始hosts已备份至 $BACKUP_PATH（每次运行会覆盖旧备份）"
echo -e "2. 若访问GitHub仍有问题，可恢复备份：$SUDO_CMD cp $BACKUP_PATH $HOSTS_PATH"
echo -e "3. 建议重启浏览器/终端后测试访问GitHub"
