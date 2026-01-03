#pragma once
#ifndef _JOY_TO_CMD_H_
#define _JOY_TO_CMD_H_

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

namespace joy_to_cmd
{
    enum ControlType
    {
        JOY_IDLE = 0,
        JOY_ENBLE,
        JOY_SPEED,
        JOY_ARM,
        JOY_STOP
    };

    class JoyToCmd
    {
    public:
        JoyToCmd(
            rclcpp::Node::SharedPtr node_handle,
            const std::string &topic_name,
            float linear,
            float angular);

        ~JoyToCmd() {}

        void JoyHandler(const sensor_msgs::msg::Joy::SharedPtr msg);
        void TimeHandler();

        int Sign0(float input)
        {
            if (input > 0)
                return 1;
            if (input < 0)
                return -1;
            return 0;
        }

    private:
        rclcpp::Node::SharedPtr nh;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel;
        rclcpp::TimerBase::SharedPtr timer;

        std::string topic_name;
        volatile ControlType joy_type = JOY_IDLE;
        ControlType last_joy_type = JOY_IDLE;
        geometry_msgs::msg::Twist cmd;
        float max_linear_speed;
        float max_angular_speed;
        float linear_speed;
        float angular_speed;
    };

}

#endif