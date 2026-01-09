#pragma once
#ifndef _base_driver_H_
#define _base_driver_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <can_msgs/Frame.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

#include <thread>
#include <cmath>
#include <mutex>
#include <atomic>
#include <chrono>

namespace base_driver
{
    class BaseDriver
    {
    public:
        BaseDriver(ros::NodeHandle &nhandle);
        ~BaseDriver();

        void controlLoop();
        void sendCanopenSDO(uint8_t node_id, uint16_t index, uint8_t subindex, int32_t value, bool is_write);
        void motorDataCallback(const can_msgs::FrameConstPtr msg);
        void cmdVelCallback(const geometry_msgs::Twist &msg);
        void publishOdom();
        void timer1HzCallback(const ros::TimerEvent &);
        bool initCanopenNodes();
        bool checkPowerStatus();
        std::string parseStatusWord(uint16_t status);

    private:
        ros::NodeHandle nh;
        ros::Publisher pub_can_msg;
        ros::Publisher pub_odom;
        ros::Publisher pub_motor_state;
        ros::Subscriber sub_can_msg;
        ros::Subscriber sub_cmd_vel;
        ros::Timer timer_1hz;

        std::shared_ptr<std::thread> control_thread;
        std::mutex mtx_motor_state;
        std::mutex mtx_odom;
        std::mutex mtx_cmd;
        std::atomic<bool> thread_running;
        std::atomic<bool> motor_enabled;

        int left_motor_node_id;
        int right_motor_node_id;
        double wheel_radius;
        double wheel_base;
        double gear_ratio;
        double max_speed_rpm;
        double linear_scale;
        double angular_scale;
        bool pub_tf;
        int control_rate;
        double power_timeout;
        double sdo_send_timeout;

        ros::Time last_feedback_time;
        ros::Time last_odom_time;
        uint16_t left_motor_status;
        uint16_t right_motor_status;
        bool left_motor_fault;
        bool right_motor_fault;
        bool can_bus_error;

        double odom_x;
        double odom_y;
        double odom_theta;
        double linear_speed;
        double angular_speed;
        int16_t left_motor_rpm;
        int16_t right_motor_rpm;
        geometry_msgs::Twist last_cmd_vel;

        tf2_ros::TransformBroadcaster tf_broadcaster;

        static const uint16_t CTRL_WORD = 0x6040;
        static const uint16_t OP_MODE = 0x6060;
        static const uint16_t SPEED_CMD = 0x60FF;
        static const uint16_t SPEED_ACT = 0x606C;
        static const uint16_t STATUS_WORD = 0x6041;
        static const uint16_t FAULT_RESET = 0x6040;

        static const uint16_t SW_READY = 0x0001;
        static const uint16_t SW_SWITCHED_ON = 0x0002;
        static const uint16_t SW_OP_ENABLE = 0x0004;
        static const uint16_t SW_FAULT = 0x0008;
        static const uint16_t SW_VOLTAGE_ENABLE = 0x0010;
        static const uint16_t SW_QUICK_STOP = 0x0020;
        static const uint16_t SW_SWITCH_ON_DIS = 0x0040;
        static const uint16_t SW_WARNING = 0x0080;
        static const uint16_t SW_INIT_SUCCESS = 0x0007;

        static const int32_t CW_SHUTDOWN = 0x00000006;
        static const int32_t CW_SWITCH_ON = 0x00000007;
        static const int32_t CW_OP_ENABLE = 0x0000000F;
        static const int32_t CW_FAULT_RESET = 0x00000080;
        static const int32_t CW_QUICK_STOP = 0x00000000;
    };
}

#endif