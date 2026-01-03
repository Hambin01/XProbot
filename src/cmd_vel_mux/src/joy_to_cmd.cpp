#include "joy_to_cmd.h"
#include <signal.h>
#include <chrono>

namespace joy_to_cmd
{
    JoyToCmd::JoyToCmd(
        rclcpp::Node::SharedPtr node_handle,
        const std::string &topic_name,
        float linear,
        float angular)
        : nh(node_handle),
          topic_name(topic_name),
          max_linear_speed(linear),
          max_angular_speed(angular)
    {
        linear_speed = max_linear_speed;
        angular_speed = max_angular_speed;

        RCLCPP_INFO(nh->get_logger(), "JoyToCmd 初始化成功！");
        RCLCPP_INFO(nh->get_logger(), "  - 发布话题: %s", topic_name.c_str());
        RCLCPP_INFO(nh->get_logger(), "  - 最大线速度: %.2f m/s", max_linear_speed);
        RCLCPP_INFO(nh->get_logger(), "  - 最大角速度: %.2f rad/s", max_angular_speed);

        sub_joy = nh->create_subscription<sensor_msgs::msg::Joy>(
            "joy",
            1,
            std::bind(&JoyToCmd::JoyHandler, this, std::placeholders::_1));

        pub_cmd_vel = nh->create_publisher<geometry_msgs::msg::Twist>(
            topic_name,
            1);

        timer = nh->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&JoyToCmd::TimeHandler, this));
    }

    void JoyToCmd::JoyHandler(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->header.frame_id != topic_name)
            return;

        if ((fabs(msg->axes[2]) == 1 && fabs(msg->axes[5]) == 0) ||
            (fabs(msg->axes[2]) == 0 && fabs(msg->axes[5]) == 1) ||
            (fabs(msg->axes[2]) == 1 && fabs(msg->axes[5]) == 1) ||
            (fabs(msg->axes[2]) == 0 && fabs(msg->axes[5]) == 0))
        {
            joy_type = JOY_IDLE;
            return;
        }

        joy_type = JOY_ENBLE;

        if (msg->buttons[6] != 1)
        {
            if (fabs(msg->axes[2]) < 1)
            {
                cmd.linear.x = msg->axes[1] * linear_speed;
                cmd.angular.z = msg->axes[3] * angular_speed;

                cmd.linear.x = fabs(cmd.linear.x) <= max_linear_speed ? cmd.linear.x : Sign0(cmd.linear.x) * max_linear_speed;
                cmd.angular.z = fabs(cmd.angular.z) <= max_angular_speed ? cmd.angular.z : Sign0(cmd.angular.z) * max_angular_speed;

                joy_type = JOY_SPEED;
                return;
            }

            if (fabs(msg->axes[5]) < 1 && fabs(msg->axes[1]) != 0)
            {
                joy_type = JOY_ARM;
                if (msg->buttons[0] == 1)
                    RCLCPP_INFO(nh->get_logger(), "机械臂1动作");
                if (msg->buttons[1] == 1)
                    RCLCPP_INFO(nh->get_logger(), "机械臂2动作");
                if (msg->buttons[2] == 1)
                    RCLCPP_INFO(nh->get_logger(), "机械臂3动作");
                if (msg->buttons[3] == 1)
                    RCLCPP_INFO(nh->get_logger(), "机械臂4动作");
                if (msg->buttons[4] == 1)
                    RCLCPP_INFO(nh->get_logger(), "机械臂5动作");
                if (msg->buttons[5] == 1)
                    RCLCPP_INFO(nh->get_logger(), "机械臂6动作");

                return;
            }
        }
        else
        {
            joy_type = JOY_STOP;
            RCLCPP_WARN(nh->get_logger(), "急停按钮被按下！");
        }

        cmd.linear.x = 0;
        cmd.angular.z = 0;
        linear_speed = max_linear_speed;
        angular_speed = max_angular_speed;
    }

    void JoyToCmd::TimeHandler()
    {
        switch (joy_type)
        {
        case JOY_SPEED:
            pub_cmd_vel->publish(cmd);
            last_joy_type = joy_type;
            break;

        case JOY_ARM:
            last_joy_type = joy_type;
            break;

        case JOY_STOP:
            cmd.linear.x = 0;
            cmd.angular.z = 0;
            pub_cmd_vel->publish(cmd);
            last_joy_type = joy_type;
            break;

        case JOY_IDLE:
            if (last_joy_type != JOY_IDLE)
            {
                cmd.linear.x = 0;
                cmd.angular.z = 0;
                pub_cmd_vel->publish(cmd);
            }
            last_joy_type = joy_type;
            break;

        default:
            break;
        }
    }

}
