#include "cmd_vel_mux.h"
#include <signal.h>
#include <chrono>

namespace cmd_vel_mux
{
    CmdVelMux::CmdVelMux(rclcpp::Node::SharedPtr &node_handle)
        : nh(node_handle)
    {
        priority = std::make_shared<int>(init_priority);
        timeout = std::make_shared<float>(init_timeout);
        change_time = std::make_shared<rclcpp::Time>(nh->now());

        std::string yaml_file;
        nh->declare_parameter<std::string>("yaml_cfg_file", "");
        if (!nh->get_parameter("yaml_cfg_file", yaml_file))
        {
            RCLCPP_ERROR(nh->get_logger(), "未找到参数 yaml_cfg_file！");
        }
        if (!ImportParams(yaml_file))
            return;

        timer = nh->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CmdVelMux::TimeHandler, this));
    }

    void CmdVelMux::TimeHandler()
    {
        if ((nh->now() - *change_time).seconds() > *timeout && *priority != init_priority)
        {
            *priority = init_priority;
            *timeout = init_timeout;
            *change_time = nh->now();
        }
    }

    bool CmdVelMux::ImportParams(std::string yaml_file)
    {
        try
        {
            YAML::Node doc;
            doc = YAML::LoadFile(yaml_file);

            cmd_topic_name = doc["publisher"].as<std::string>();

            for (size_t i = 0; i < doc["cmd_nodes"].size(); i++)
            {
                std::shared_ptr<cmd_node::CmdNode> node = std::make_shared<cmd_node::CmdNode>(
                    nh,
                    doc["cmd_nodes"][i]["topic_name"].as<std::string>(),
                    doc["cmd_nodes"][i]["priority"].as<int>(),
                    doc["cmd_nodes"][i]["timeout"].as<float>(),
                    doc["cmd_nodes"][i]["short_desc"].as<std::string>(),
                    priority,
                    timeout,
                    change_time,
                    cmd_topic_name);

                cmd_nodes.push_back(node);
            }

            for (size_t i = 0; i < doc["joy_types"].size(); i++)
            {
                std::shared_ptr<joy_to_cmd::JoyToCmd> joy_node = std::make_shared<joy_to_cmd::JoyToCmd>(
                    nh,
                    doc["joy_types"][i]["joy_frame"].as<std::string>(),
                    doc["joy_types"][i]["max_linear_speed"].as<float>(),
                    doc["joy_types"][i]["max_angular_speed"].as<float>());
                joys_nodes.push_back(joy_node);
            }

            RCLCPP_INFO(nh->get_logger(), "\033[1;32m----> cmd_vel_mux import params ok.\033[0m");
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            return false;
        }

        return true;
    }

}

static void Handler(int sig)
{
    rclcpp::shutdown();
    exit(0);
}
int main(int argc, char **argv)
{
    try
    {
        rclcpp::init(argc, argv);
        auto nh = std::make_shared<rclcpp::Node>("cmd_vel_mux");
        signal(SIGINT, Handler);
        cmd_vel_mux::CmdVelMux cmd(nh);
        RCLCPP_INFO(nh->get_logger(), "\033[1;32m----> cmd_vel_mux node Started.\033[0m");
        rclcpp::spin(nh);
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("cmd_vel_mux"), "节点启动崩溃: %s", e.what());
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::shutdown();
    return 0;
}
