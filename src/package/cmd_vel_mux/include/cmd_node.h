/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-03-14 09:01:36
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-03-15 09:35:12
 * @FilePath: /can_package/src/cmd_vel_mux/include/cmd_node.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#pragma once
#ifndef _CMD_NODE_H_
#define _CMD_NODE_H_


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

namespace cmd_node
{


class CmdNode
{

public:
    CmdNode(ros::NodeHandle &nhandle,
            std::string topic_name,
            int pri,
            float time,
            std::string short_desc,
            std::shared_ptr<int> g_pri,
            std::shared_ptr<float> g_timeout,
            std::shared_ptr<ros::Time> g_change_time,
            std::string out_topic);
    ~CmdNode(){}
    void CmdHandler(const geometry_msgs::TwistConstPtr msg);

public:
    std::shared_ptr<int> global_priority;
    std::shared_ptr<float> global_timeout;
    std::shared_ptr<ros::Time> global_change_time;

private:
    ros::NodeHandle nh;
    std::string name;
    std::string cmd_topic_name;
    float timeout;
    int priority;
    std::string short_desc;
    ros::Subscriber sub_cmd;
    ros::Publisher  pub_cmd_vel;
};
}

#endif
