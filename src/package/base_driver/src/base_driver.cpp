#include "base_driver.h"
#include <signal.h>
#include <cmath>
#include <iomanip>
#include <sstream>

using namespace tf2;
using namespace std::chrono;

namespace base_driver
{

    BaseDriver::BaseDriver(ros::NodeHandle &nhandle)
    {

        nh = nhandle;
        ros::NodeHandle nh_p("~");

        nh_p.param<int>("left_motor_node_id", left_motor_node_id, 1);
        nh_p.param<int>("right_motor_node_id", right_motor_node_id, 2);
        nh_p.param<double>("wheel_radius", wheel_radius, 0.295);
        nh_p.param<double>("wheel_base", wheel_base, 0.73);
        nh_p.param<double>("gear_ratio", gear_ratio, 16.0);
        nh_p.param<double>("max_speed_rpm", max_speed_rpm, 1500.0);
        nh_p.param<double>("linear_scale", linear_scale, 1.0);
        nh_p.param<double>("angular_scale", angular_scale, 1.0);
        nh_p.param<bool>("pub_tf", pub_tf, false);
        nh_p.param<int>("control_rate", control_rate, 25);
        nh_p.param<double>("power_timeout", power_timeout, 3.0);
        nh_p.param<double>("sdo_send_timeout", sdo_send_timeout, 0.1);

        if (wheel_radius <= 0 || wheel_base <= 0 || gear_ratio <= 0)
        {
            ROS_FATAL("[%s] Invalid physical parameters! Radius=%.3f, Base=%.3f, Ratio=%.1f",
                      __FUNCTION__, wheel_radius, wheel_base, gear_ratio);
            ros::shutdown();
            return;
        }
        if (left_motor_node_id < 1 || left_motor_node_id > 127 || right_motor_node_id < 1 || right_motor_node_id > 127)
        {
            ROS_FATAL("[%s] Invalid node ID (1-127)! Left=%d, Right=%d", __FUNCTION__, left_motor_node_id, right_motor_node_id);
            ros::shutdown();
            return;
        }

        sub_can_msg = nh.subscribe("received_can_message_driver", 50, &BaseDriver::motorDataCallback, this, ros::TransportHints().tcpNoDelay());
        sub_cmd_vel = nh.subscribe("cmd_vel", 10, &BaseDriver::cmdVelCallback, this, ros::TransportHints().tcpNoDelay());
        pub_can_msg = nh.advertise<can_msgs::Frame>("send_can_message_driver", 50);
        pub_odom = nh.advertise<nav_msgs::Odometry>("base_odom", 10);
        pub_motor_state = nh.advertise<std_msgs::Bool>("motor_enabled", 10);

        timer_1hz = nh.createTimer(ros::Duration(1.0), &BaseDriver::timer1HzCallback, this);
        thread_running = true;
        motor_enabled = false;
        can_bus_error = false;
        left_motor_fault = false;
        right_motor_fault = false;

        last_feedback_time = ros::Time::now();
        last_odom_time = ros::Time::now();
        left_motor_status = 0;
        right_motor_status = 0;

        try
        {
            control_thread = std::make_shared<std::thread>(&BaseDriver::controlLoop, this);
            if (control_thread->joinable())
                control_thread->detach();
        }
        catch (const std::exception &e)
        {
            ROS_FATAL("[%s] Failed to start control thread: %s", __FUNCTION__, e.what());
            ros::shutdown();
            return;
        }

        ROS_INFO("[%s] CANopen SDO Driver Initialized (Control Rate: %dHz, Timeout: %.1fs)",
                 __FUNCTION__, control_rate, power_timeout);
    }

    BaseDriver::~BaseDriver()
    {
        thread_running = false;
        motor_enabled = false;

        sendCanopenSDO(left_motor_node_id, CTRL_WORD, 0, CW_QUICK_STOP, true);
        sendCanopenSDO(right_motor_node_id, CTRL_WORD, 0, CW_QUICK_STOP, true);
        ros::Duration(0.5).sleep();

        if (control_thread && control_thread->joinable())
            control_thread->join();

        ROS_INFO("[%s] Driver stopped, motors are in safe state", __FUNCTION__);
    }

    void BaseDriver::sendCanopenSDO(uint8_t node_id, uint16_t index, uint8_t subindex, int32_t value, bool is_write)
    {
        if (node_id < 1 || node_id > 127)
        {
            ROS_WARN("[%s] Invalid node ID: %d", __FUNCTION__, node_id);
            return;
        }

        can_msgs::Frame msg;
        msg.header.stamp = ros::Time::now();
        msg.id = 0x600 + node_id;
        msg.dlc = 8;
        msg.is_extended = false;
        msg.is_rtr = false;
        msg.is_error = false;

        msg.data[0] = is_write ? 0x23 : 0x40;
        msg.data[1] = (index >> 8) & 0xFF;
        msg.data[2] = index & 0xFF;
        msg.data[3] = subindex;
        msg.data[4] = (value >> 0) & 0xFF;
        msg.data[5] = (value >> 8) & 0xFF;
        msg.data[6] = (value >> 16) & 0xFF;
        msg.data[7] = (value >> 24) & 0xFF;

        try
        {
            pub_can_msg.publish(msg);
            ros::Duration(sdo_send_timeout).sleep();
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("[%s] Failed to send SDO (Node %d, Index 0x%04X): %s",
                      __FUNCTION__, node_id, index, e.what());
            can_bus_error = true;
        }
    }

    bool BaseDriver::initCanopenNodes()
    {
        std::lock_guard<std::mutex> lock(mtx_motor_state);
        left_motor_status = 0;
        right_motor_status = 0;
        left_motor_fault = false;
        right_motor_fault = false;

        ROS_INFO("[%s] Starting motor initialization (Node L:%d, R:%d)...",
                 __FUNCTION__, left_motor_node_id, right_motor_node_id);

        sendCanopenSDO(left_motor_node_id, CTRL_WORD, 0, CW_FAULT_RESET, true);
        sendCanopenSDO(right_motor_node_id, CTRL_WORD, 0, CW_FAULT_RESET, true);
        ros::Duration(0.2).sleep();

        sendCanopenSDO(left_motor_node_id, OP_MODE, 0, 3, true);
        sendCanopenSDO(right_motor_node_id, OP_MODE, 0, 3, true);
        ros::Duration(0.1).sleep();

        sendCanopenSDO(left_motor_node_id, CTRL_WORD, 0, CW_SHUTDOWN, true);
        sendCanopenSDO(right_motor_node_id, CTRL_WORD, 0, CW_SHUTDOWN, true);
        ros::Duration(0.1).sleep();

        sendCanopenSDO(left_motor_node_id, CTRL_WORD, 0, CW_SWITCH_ON, true);
        sendCanopenSDO(right_motor_node_id, CTRL_WORD, 0, CW_SWITCH_ON, true);
        ros::Duration(0.1).sleep();

        sendCanopenSDO(left_motor_node_id, CTRL_WORD, 0, CW_OP_ENABLE, true);
        sendCanopenSDO(right_motor_node_id, CTRL_WORD, 0, CW_OP_ENABLE, true);
        ros::Duration(0.3).sleep();

        sendCanopenSDO(left_motor_node_id, STATUS_WORD, 0, 0, false);
        sendCanopenSDO(right_motor_node_id, STATUS_WORD, 0, 0, false);
        ros::Duration(0.2).sleep();

        bool left_ok = (left_motor_status & SW_INIT_SUCCESS) == SW_INIT_SUCCESS && !(left_motor_status & SW_FAULT);
        bool right_ok = (right_motor_status & SW_INIT_SUCCESS) == SW_INIT_SUCCESS && !(right_motor_status & SW_FAULT);

        if (left_ok && right_ok)
        {
            motor_enabled = true;
            last_feedback_time = ros::Time::now();
            ROS_INFO("[%s] Motors initialized successfully! L: %s, R: %s",
                     __FUNCTION__, parseStatusWord(left_motor_status).c_str(), parseStatusWord(right_motor_status).c_str());

            std_msgs::Bool enable_msg;
            enable_msg.data = true;
            pub_motor_state.publish(enable_msg);
            return true;
        }
        else
        {
            motor_enabled = false;
            ROS_WARN("[%s] Initialization failed! L: %s, R: %s",
                     __FUNCTION__, parseStatusWord(left_motor_status).c_str(), parseStatusWord(right_motor_status).c_str());

            std_msgs::Bool enable_msg;
            enable_msg.data = false;
            pub_motor_state.publish(enable_msg);
            return false;
        }
    }

    bool BaseDriver::checkPowerStatus()
    {
        std::lock_guard<std::mutex> lock(mtx_motor_state);
        double duration = (ros::Time::now() - last_feedback_time).toSec();

        if (can_bus_error)
        {
            ROS_WARN("[%s] CAN bus error detected", __FUNCTION__);
            motor_enabled = false;
            can_bus_error = false;
            return false;
        }

        if (duration > power_timeout)
        {
            ROS_WARN("[%s] Power loss/communication timeout (%.1fs > %.1fs)",
                     __FUNCTION__, duration, power_timeout);
            motor_enabled = false;
            return false;
        }

        if (left_motor_fault || right_motor_fault)
        {
            ROS_WARN("[%s] Motor fault detected! L:%d, R:%d", __FUNCTION__, left_motor_fault, right_motor_fault);
            motor_enabled = false;
            return false;
        }

        return true;
    }

    void BaseDriver::timer1HzCallback(const ros::TimerEvent &)
    {
        if (!checkPowerStatus())
        {
            initCanopenNodes();
            return;
        }

        if (motor_enabled)
        {
            sendCanopenSDO(left_motor_node_id, STATUS_WORD, 0, 0, false);
            sendCanopenSDO(right_motor_node_id, STATUS_WORD, 0, 0, false);

            std::lock_guard<std::mutex> lock(mtx_motor_state);
            bool left_ok = (left_motor_status & SW_INIT_SUCCESS) == SW_INIT_SUCCESS && !(left_motor_status & SW_FAULT);
            bool right_ok = (right_motor_status & SW_INIT_SUCCESS) == SW_INIT_SUCCESS && !(right_motor_status & SW_FAULT);

            if (!left_ok || !right_ok)
            {
                ROS_WARN("[%s] Motor status abnormal, reinitializing...", __FUNCTION__);
                motor_enabled = false;
                initCanopenNodes();
            }
        }
        else
        {
            static ros::Time last_retry_time = ros::Time::now();
            if ((ros::Time::now() - last_retry_time).toSec() > power_timeout)
            {
                initCanopenNodes();
                last_retry_time = ros::Time::now();
            }
        }
    }

    void BaseDriver::controlLoop()
    {
        ros::Rate loop_rate(control_rate);
        while (ros::ok() && thread_running)
        {
            try
            {
                if (motor_enabled)
                {
                    std::lock_guard<std::mutex> lock(mtx_motor_state);
                    sendCanopenSDO(left_motor_node_id, SPEED_ACT, 0, 0, false);
                    sendCanopenSDO(right_motor_node_id, SPEED_ACT, 0, 0, false);

                    publishOdom();
                }
                loop_rate.sleep();
            }
            catch (const std::exception &e)
            {
                ROS_ERROR("[%s] Control loop exception: %s", __FUNCTION__, e.what());
                motor_enabled = false;
                ros::Duration(0.5).sleep();
            }
        }
        ROS_INFO("[%s] Control loop stopped", __FUNCTION__);
    }

    void BaseDriver::motorDataCallback(const can_msgs::FrameConstPtr msg)
    {
        if (!msg || msg->dlc != 8 || msg->is_error || msg->is_rtr)
        {
            can_bus_error = true;
            return;
        }

        int node_id = msg->id - 0x580;
        if (node_id != left_motor_node_id && node_id != right_motor_node_id)
            return;

        uint16_t index = (msg->data[1] << 8) | msg->data[2];
        uint8_t subindex = msg->data[3];

        std::lock_guard<std::mutex> lock(mtx_motor_state);
        last_feedback_time = ros::Time::now();

        if (index == STATUS_WORD && subindex == 0)
        {
            uint16_t status = (msg->data[5] << 8) | msg->data[4];
            if (node_id == left_motor_node_id)
            {
                left_motor_status = status;
                left_motor_fault = (status & SW_FAULT) != 0;
            }
            else if (node_id == right_motor_node_id)
            {
                right_motor_status = status;
                right_motor_fault = (status & SW_FAULT) != 0;
            }
            return;
        }

        if (index == SPEED_ACT && subindex == 0)
        {
            int32_t rpm = (msg->data[7] << 24) | (msg->data[6] << 16) | (msg->data[5] << 8) | msg->data[4];
            rpm = std::clamp(rpm, (int32_t)-max_speed_rpm * 2, (int32_t)max_speed_rpm * 2);

            if (node_id == left_motor_node_id)
                left_motor_rpm = static_cast<int16_t>(rpm);
            else if (node_id == right_motor_node_id)
                right_motor_rpm = static_cast<int16_t>(rpm);
            return;
        }
    }

    void BaseDriver::cmdVelCallback(const geometry_msgs::Twist &msg)
    {
        if (!motor_enabled)
            return;

        std::lock_guard<std::mutex> lock(mtx_cmd);
        last_cmd_vel = msg;

        double linear_x = std::clamp(msg.linear.x, -1.0 * max_speed_rpm * M_PI * wheel_radius / (30 * gear_ratio),
                                     1.0 * max_speed_rpm * M_PI * wheel_radius / (30 * gear_ratio));
        double angular_z = std::clamp(msg.angular.z, -2.0 * M_PI, 2.0 * M_PI);

        double v_left = linear_x + (angular_z * wheel_base / 2.0);
        double v_right = linear_x - (angular_z * wheel_base / 2.0);

        double rpm_left = (v_left * 60 * gear_ratio) / (2 * M_PI * wheel_radius);
        double rpm_right = (v_right * 60 * gear_ratio) / (2 * M_PI * wheel_radius);

        rpm_left = std::clamp(rpm_left, -max_speed_rpm, max_speed_rpm);
        rpm_right = std::clamp(rpm_right, -max_speed_rpm, max_speed_rpm);

        sendCanopenSDO(left_motor_node_id, SPEED_CMD, 0, static_cast<int32_t>(rpm_left), true);
        sendCanopenSDO(right_motor_node_id, SPEED_CMD, 0, static_cast<int32_t>(rpm_right), true);
    }

    void BaseDriver::publishOdom()
    {
        std::lock_guard<std::mutex> lock_odom(mtx_odom);
        std::lock_guard<std::mutex> lock_motor(mtx_motor_state);

        ros::Time now = ros::Time::now();
        double dt = (now - last_odom_time).toSec();
        if (dt <= 0.001 || dt > 1.0)
        {
            last_odom_time = now;
            return;
        }

        double v_left = (left_motor_rpm / 60.0) * 2 * M_PI * wheel_radius / gear_ratio;
        double v_right = (right_motor_rpm / 60.0) * 2 * M_PI * wheel_radius / gear_ratio;

        linear_speed = (v_left + v_right) / 2.0 * linear_scale;
        angular_speed = (v_right - v_left) / wheel_base * angular_scale;

        double dx = linear_speed * cos(odom_theta) * dt;
        double dy = linear_speed * sin(odom_theta) * dt;
        double dtheta = angular_speed * dt;

        odom_x += dx;
        odom_y += dy;
        odom_theta = fmod(odom_theta + dtheta, 2 * M_PI);
        if (odom_theta < 0)
            odom_theta += 2 * M_PI;

        if (pub_tf)
        {
            geometry_msgs::TransformStamped tf_msg;
            tf_msg.header.stamp = now;
            tf_msg.header.frame_id = "odom";
            tf_msg.child_frame_id = "base_link";
            tf_msg.transform.translation.x = odom_x;
            tf_msg.transform.translation.y = odom_y;
            tf_msg.transform.translation.z = 0.0;
            tf_msg.transform.rotation = toMsg(Quaternion(Vector3(0, 0, 1), odom_theta));
            tf_broadcaster.sendTransform(tf_msg);
        }

        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = odom_x;
        odom_msg.pose.pose.position.y = odom_y;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = toMsg(Quaternion(Vector3(0, 0, 1), odom_theta));

        odom_msg.twist.twist.linear.x = linear_speed;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.angular.z = angular_speed;

        odom_msg.pose.covariance = {
            0.001, 0, 0, 0, 0, 0,
            0, 0.001, 0, 0, 0, 0,
            0, 0, 1e9, 0, 0, 0,
            0, 0, 0, 1e9, 0, 0,
            0, 0, 0, 0, 1e9, 0,
            0, 0, 0, 0, 0, 0.001};
        odom_msg.twist.covariance = odom_msg.pose.covariance;

        pub_odom.publish(odom_msg);
        last_odom_time = now;
    }

    std::string BaseDriver::parseStatusWord(uint16_t status)
    {
        std::ostringstream oss;
        oss << "0x" << std::hex << std::setw(4) << std::setfill('0') << status << " [";

        if (status & SW_FAULT)
            oss << "FAULT, ";
        if (status & SW_OP_ENABLE)
            oss << "OP_ENABLE, ";
        if (status & SW_SWITCHED_ON)
            oss << "SWITCHED_ON, ";
        if (status & SW_READY)
            oss << "READY, ";
        if (status & SW_VOLTAGE_ENABLE)
            oss << "VOLTAGE_ON, ";
        if (status & SW_QUICK_STOP)
            oss << "QUICK_STOP, ";
        if (status & SW_SWITCH_ON_DIS)
            oss << "SWITCH_ON_DIS, ";
        if (status & SW_WARNING)
            oss << "WARNING, ";

        std::string str = oss.str();
        if (str.back() == ',' && str[str.size() - 2] == ' ')
            str = str.substr(0, str.size() - 2);
        str += "]";
        return str;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "base_driver", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    try
    {
        base_driver::BaseDriver driver(nh);
        ROS_INFO("\033[1;32m===== CANopen SDO Industrial Driver Started =====\033[0m");

        ros::AsyncSpinner spinner(2);
        spinner.start();
        ros::waitForShutdown();
        spinner.stop();
    }
    catch (const std::exception &e)
    {
        ROS_FATAL("Driver initialization failed: %s", e.what());
        return -1;
    }

    return 0;
}