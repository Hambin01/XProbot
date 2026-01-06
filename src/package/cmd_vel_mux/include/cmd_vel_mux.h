/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-03-13 14:30:58
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-03-15 09:38:11
 * @FilePath: /can_package/src/cmd_vel_mux/include/cmd_vel_mux.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#pragma once
#ifndef _CMD_VEL_MUX_H_
#define _CMD_VEL_MUX_H_


#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/Twist.h>
#include "cmd_node.h"
#include "joy_to_cmd.h"

namespace cmd_vel_mux
{

class CmdVelMux
{


public:
    CmdVelMux(ros::NodeHandle &nhandle);
    ~CmdVelMux(){}

    bool ImportParams(std::string yaml_file);
    void TimeHandler(const ros::TimerEvent& event);

public:
    std::vector<std::shared_ptr<cmd_node::CmdNode> > cmd_nodes;
    std::vector<std::shared_ptr<joy_to_cmd::JoyToCmd>> joys_nodes;
     
    std::string cmd_topic_name;
    

private:
    ros::NodeHandle nh;
    ros::Timer timer;
    std::shared_ptr<float> timeout;
    std::shared_ptr<int> priority;
    std::shared_ptr<ros::Time> change_time;
    int init_priority = 9999;
    float init_timeout = 0;
};
}

#endif
