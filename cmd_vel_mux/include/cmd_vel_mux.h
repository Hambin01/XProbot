#pragma once
#ifndef _CMD_VEL_MUX_H_
#define _CMD_VEL_MUX_H_

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

#include "cmd_node.h"
#include "joy_to_cmd.h"

namespace cmd_vel_mux
{
    class CmdVelMux
    {
    public:
        CmdVelMux(rclcpp::Node::SharedPtr &node_handle);
        ~CmdVelMux() {}

        bool ImportParams(std::string yaml_file);
        void TimeHandler();

    public:
        std::vector<std::shared_ptr<cmd_node::CmdNode>> cmd_nodes;
        std::vector<std::shared_ptr<joy_to_cmd::JoyToCmd>> joys_nodes;

        std::string cmd_topic_name;

    private:
        rclcpp::Node::SharedPtr nh;
        rclcpp::TimerBase::SharedPtr timer;
        std::shared_ptr<float> timeout;
        std::shared_ptr<int> priority;
        std::shared_ptr<rclcpp::Time> change_time;
        int init_priority = 9999;
        float init_timeout = 0;
    };
}

#endif