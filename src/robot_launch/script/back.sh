#!/bin/sh


cd /

sudo tar -czvf /robot.tgz --exclude=/proc --exclude=/tmp --exclude=/home.tgz --exclude=/media --exclude=/lost+found --exclude=/robot.tgz --exclude=/mnt --exclude=/sys --exclude=/etc/fstab --exclude=/cdrom --exclude=/data --exclude=/boot --exclude=/dev --exclude=/home --exclude=/run --exclude=/initrd.img --exclude=/initrd.img.old --exclude=/vmlinuz /

sudo tar -czvf /home.tgz --exclude=/home.tgz --exclude=/robot.tgz /home

# 拷贝文件到磁盘
echo -e "\033[44;37;5m #=========== Begin to copy file to disk! ===========#\033[0m"
sleep 5s

sudo mv /home.tgz /media/b1/hambin/b1
sudo mv /robot.tgz /media/b1/hambin/b1
sync
sync

echo -e "\033[44;37;5m #=========== backup successful! ===========#\033[0m"
