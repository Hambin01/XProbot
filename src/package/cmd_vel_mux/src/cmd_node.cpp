/*
 * @Author: hambin
 * @Date: 2023-03-10 13:40:12
 * @LastEditors: Please set LastEditors
 * @FilePath: src/cmd_vel_mux.cpp
 * @Description: can to joy message
 */
#include "cmd_node.h"
#include <signal.h>

namespace cmd_node
{
    CmdNode::CmdNode(ros::NodeHandle &nhandle,
                     std::string topic_name,
                     int pri,
                     float time,
                     std::string short_desc,
                     std::shared_ptr<int> g_pri,
                     std::shared_ptr<float> g_timeout,
                     std::shared_ptr<ros::Time> g_change_time,
                     std::string out_topic):
                     nh(nhandle),
                     name(topic_name),
                     priority(pri),
                     timeout(time),
                     global_priority(g_pri),
                     global_timeout(g_timeout),
                     global_change_time(g_change_time),
                     cmd_topic_name(out_topic)

    {
        //订阅输入速度
        sub_cmd = nh.subscribe(topic_name, 1, &CmdNode::CmdHandler, this);
        //发布输出速度
        pub_cmd_vel = nh.advertise<geometry_msgs::Twist>(cmd_topic_name, 1);
    }



    void CmdNode::CmdHandler(const geometry_msgs::TwistConstPtr msg)
    {
        // 修改优先级
        if (priority <= *global_priority)
        {
            *global_priority = priority;
            *global_timeout = timeout;
            *global_change_time = ros::Time::now();
            // 发布命令
            pub_cmd_vel.publish(*msg);
            //std::cout<<"pub_cmd: "<<name<<"  g_pri: "<<*global_priority<<" pri: "<<priority<<std::endl;
        }
    }




} 

