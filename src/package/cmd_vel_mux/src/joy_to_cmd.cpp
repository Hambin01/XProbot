/*
 * @Author: hambin
 * @Date: 2023-03-10 13:40:12
 * @LastEditors: Please set LastEditors
 * @FilePath: src/cmd_vel_mux.cpp
 * @Description: can to joy message
 */
#include "joy_to_cmd.h"
#include <signal.h>

namespace joy_to_cmd
{
    JoyToCmd::JoyToCmd(ros::NodeHandle &nhandle,
                       std::string name,
                       float linear,
                       float angular)
    {
        nh = nhandle;
        topic_name = name;
        max_linear_speed = linear;
        max_angular_speed = angular;
        linear_speed = max_linear_speed ;//* 0.5;
        angular_speed = max_angular_speed ;//* 0.5;

        // 订阅控制器话题
        sub_joy = nh.subscribe("joy", 1, &JoyToCmd::JoyHandler, this);

        // 发布控制器速度命令
        pub_cmd_vel = nh.advertise<geometry_msgs::Twist>(topic_name, 1);
        timer = nh.createTimer(ros::Duration(0.05), &JoyToCmd::TimeHandler, this);
    }

    void JoyToCmd::JoyHandler(const sensor_msgs::JoyConstPtr msg)
    {

        // 是否是当前节点的joy
        if (msg->header.frame_id != topic_name)
            return;


        // 判断手柄是否归位
        if ((fabs(msg->axes[2]) == 1 && fabs(msg->axes[5]) == 0) ||
            (fabs(msg->axes[2]) == 0 && fabs(msg->axes[5]) == 1) ||
            (fabs(msg->axes[2]) == 1 && fabs(msg->axes[5]) == 1) ||
            (fabs(msg->axes[2]) == 0 && fabs(msg->axes[5]) == 0))
        {
            joy_type = JOY_IDLE;
            return;
        }

        joy_type = JOY_ENBLE;
        // 急停未按下
        if (msg->buttons[6] != 1)
        {
            // 速度设置
            if (fabs(msg->axes[2]) < 1)//  && (fabs(msg->axes[1]!=0) || msg->axes[3]))
            {
                // linear_speed += (fabs(msg->axes[7]) * 0.001);
                // angular_speed += (fabs(msg->axes[6]) * 0.001);
                cmd.linear.x = msg->axes[1] * linear_speed;
                cmd.angular.z = msg->axes[3] * angular_speed;
                cmd.linear.x = fabs(cmd.linear.x) <= max_linear_speed ? cmd.linear.x : Sign0(cmd.linear.x)*max_linear_speed;
                cmd.angular.z = fabs(cmd.angular.z) <= max_angular_speed ? cmd.angular.z : Sign0(cmd.linear.z)*max_angular_speed;
                //std::cout << "线速度：" << cmd.linear.x << "  角速度：" << cmd.angular.z << std::endl;
                joy_type = JOY_SPEED;
                return;
            }

            // 机械臂设置
            if (fabs(msg->axes[5]) < 1 && fabs(msg->axes[1]!=0))
            {
                // to do
                joy_type = JOY_ARM;
                if (msg->buttons[0] == 1)
                    std::cout << "机械臂1动作" << std::endl;
                if (msg->buttons[1] == 1)
                    std::cout << "机械臂2动作" << std::endl;
                if (msg->buttons[2] == 1)
                    std::cout << "机械臂3动作" << std::endl;
                if (msg->buttons[3] == 1)
                    std::cout << "机械臂4动作" << std::endl;
                if (msg->buttons[4] == 1)
                    std::cout << "机械臂5动作" << std::endl;
                if (msg->buttons[5] == 1)
                    std::cout << "机械臂6动作" << std::endl;
                
                return;
            }
        }
        else
        {
            joy_type = JOY_STOP;
            std::cout << "急停按钮被按下！" << std::endl;
        }

        // 急停被按下或者未定义对应动作，置零位
        cmd.linear.x = 0;
        cmd.angular.z = 0;
        linear_speed = max_linear_speed ;//* 0.5;
        angular_speed = max_angular_speed;// * 0.5;
    }

    void JoyToCmd::TimeHandler(const ros::TimerEvent &event)
    {
        switch (joy_type)
        {
        case JOY_SPEED:
            pub_cmd_vel.publish(cmd);
            last_joy_type = joy_type;
            break;

        case JOY_ARM:
            // to do
            last_joy_type = joy_type;
            break;

        case JOY_STOP:
            // to do
            cmd.linear.x = 0;
            cmd.angular.z = 0;
            pub_cmd_vel.publish(cmd);
            last_joy_type = joy_type;
            break;

         case JOY_IDLE:
            if(last_joy_type !=JOY_IDLE){
                cmd.linear.x = 0;
                cmd.angular.z = 0;
                pub_cmd_vel.publish(cmd);
            }
            last_joy_type = joy_type;
            break;    

        default:
            break;
        }
    }

}
