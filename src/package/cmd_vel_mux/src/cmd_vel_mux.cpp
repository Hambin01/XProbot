/*
 * @Author: hambin
 * @Date: 2023-03-10 13:40:12
 * @LastEditors: Please set LastEditors
 * @FilePath: src/cmd_vel_mux.cpp
 * @Description: can to joy message
 */
#include "cmd_vel_mux.h"
#include <signal.h>

namespace cmd_vel_mux
{
    CmdVelMux::CmdVelMux(ros::NodeHandle &nhandle)
    {
        nh = nhandle;
        priority = std::make_shared<int>(init_priority);
        timeout = std::make_shared<float>(init_timeout);
        change_time = std::make_shared<ros::Time>(ros::Time::now());
        
        std::string yaml_file;
        nh.getParam("/cmd_vel_mux_node/yaml_cfg_file",yaml_file);
        if (!ImportParams(yaml_file))
            return;

        timer = nh.createTimer(ros::Duration(0.1), &CmdVelMux::TimeHandler, this);
    }

    void CmdVelMux::TimeHandler(const ros::TimerEvent &event)
    {
        // 超时，重置优先级
        if ((ros::Time::now() - *change_time).toSec() > *timeout && *priority != init_priority)
        {
            *priority = init_priority;
            *timeout = init_timeout;
            *change_time = ros::Time::now();
        }
    }

    bool CmdVelMux::ImportParams(std::string yaml_file)
    {
        try
        {
            YAML::Node doc;
            doc = YAML::LoadFile(yaml_file);

            // import out cmd_vel
            cmd_topic_name = doc["publisher"].as<std::string>();
            // import cmd_nodes
            for (size_t i = 0; i < doc["cmd_nodes"].size(); i++)
            {
                std::shared_ptr<cmd_node::CmdNode> node = std::make_shared<cmd_node::CmdNode>(nh,
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
            // import joys
            for (size_t i = 0; i < doc["joy_types"].size(); i++)
            {
                std::shared_ptr<joy_to_cmd::JoyToCmd> joy_node = std::make_shared<joy_to_cmd::JoyToCmd>(nh,
                                                                                                        doc["joy_types"][i]["joy_frame"].as<std::string>(),
                                                                                                        doc["joy_types"][i]["max_linear_speed"].as<float>(),
                                                                                                        doc["joy_types"][i]["max_angular_speed"].as<float>());
                joys_nodes.push_back(joy_node);
            }
             ROS_INFO("\033[1;32m----> cmd_vel_mux import params ok.\033[0m");
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            return false;
        }

        return true;
    }

} // namespace cmd_vel_mux

static void Handler(int sig)
{
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_mux");
    ros::NodeHandle nh;
    signal(SIGINT, Handler);
    cmd_vel_mux::CmdVelMux cmd(nh);

    ROS_INFO("\033[1;32m----> cmd_vel_mux node Started.\033[0m");
    ros::AsyncSpinner s(2);
    s.start();
    ros::waitForShutdown();

    return 0;
}
