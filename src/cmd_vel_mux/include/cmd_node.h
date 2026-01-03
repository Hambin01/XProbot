#pragma once
#ifndef _CMD_NODE_H_
#define _CMD_NODE_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

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

namespace cmd_node
{
    class CmdNode
    {
    public:
        CmdNode(
            rclcpp::Node::SharedPtr node_handle,
            std::string topic_name,
            int pri,
            float time,
            std::string short_desc,
            std::shared_ptr<int> g_pri,
            std::shared_ptr<float> g_timeout,
            std::shared_ptr<rclcpp::Time> g_change_time,
            std::string out_topic);

        ~CmdNode() {}

        void CmdHandler(const geometry_msgs::msg::Twist::SharedPtr msg);

    public:
        std::shared_ptr<int> global_priority;
        std::shared_ptr<float> global_timeout;
        std::shared_ptr<rclcpp::Time> global_change_time;

    private:
        rclcpp::Node::SharedPtr nh;
        std::string name;
        std::string cmd_topic_name;
        float timeout;
        int priority;
        std::string short_desc;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel;
    };
}

#endif