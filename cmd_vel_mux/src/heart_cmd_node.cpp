/*
 * @Author: hambin
 * @Date: 2023-03-10 13:40:12
 * @LastEditors: Please set LastEditors
 * @FilePath: src/cmd_vel_mux.cpp
 * @Description: can to joy message
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <signal.h>

class CmdVelMux : public rclcpp::Node
{
public:
    CmdVelMux() : Node("cmd_vel_mux")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("heart_cmd_vel", 1);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&CmdVelMux::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0;
        message.angular.z = 0;
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

static void Handler(int sig)
{
    rclcpp::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    signal(SIGINT, Handler);

    auto node = std::make_shared<CmdVelMux>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
