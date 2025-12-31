#include "cmd_node.h"
#include <signal.h>

namespace cmd_node
{
    CmdNode::CmdNode(
        rclcpp::Node::SharedPtr node_handle,
        std::string topic_name,
        int pri,
        float time,
        std::string short_desc,
        std::shared_ptr<int> g_pri,
        std::shared_ptr<float> g_timeout,
        std::shared_ptr<rclcpp::Time> g_change_time,
        std::string out_topic) : nh(node_handle),
                                 name(topic_name),
                                 priority(pri),
                                 timeout(time),
                                 global_priority(g_pri),
                                 global_timeout(g_timeout),
                                 global_change_time(g_change_time),
                                 cmd_topic_name(out_topic),
                                 short_desc(short_desc)
    {
        sub_cmd = nh->create_subscription<geometry_msgs::msg::Twist>(
            topic_name,
            1,
            std::bind(&CmdNode::CmdHandler, this, std::placeholders::_1));

        pub_cmd_vel = nh->create_publisher<geometry_msgs::msg::Twist>(
            cmd_topic_name,
            1);
    }

    void CmdNode::CmdHandler(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (priority <= *global_priority)
        {
            *global_priority = priority;
            *global_timeout = timeout;
            *global_change_time = nh->now();
            pub_cmd_vel->publish(*msg);
        }
    }
}