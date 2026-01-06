#!/bin/bash
echo "deb [trusted=yes arch=amd64] http://deb.repo.autolabor.com.cn jammy main" | sudo tee /etc/apt/sources.list.d/autolabor.list
sudo apt update
sudo apt install ros-noetic-autolabor

sudo apt install libsdl1.2-dev
sudo apt install libsdl-image1.2
pip install twisted
pip install pyopenssl
pip install autobahn
pip install tornado
pip install pymongo
pip install service_identity

git clone https://github.com/orocos/orocos_kinematics_dynamics.git
cd orocos_kinematics_dynamics/orocos_kdl
mkdir build
cd build
cmake ..
make -j4
sudo make install
sudo ln -s /usr/local/lib/liborocos-kdl.so.1.5.4 /usr/local/lib/liborocos-kdl.so.1.5.1
cd
rm -r orocos_kinematics_dynamics

sudo ln -s /opt/ros/humble/share/cartographer/configuration_files  /home/robot/XProbot/src/robot_launch/carto_file
sudo chmod 777 -R /opt/ros/humble/share/cartographer/configuration_files

sudo apt install nginx
sudo ln -s /home/robot/XProbot/src/robot_launch/script/robot_view /var/www
sudo ln -s /home/robot/XProbot/src/robot_launch/script/robot.conf /etc/nginx/conf.d/robot.conf
sudo chmod -R 755 /var/www/robot_view
sudo chmod -R 644 /var/www/robot_view/index.html
sudo systemctl restart nginx

