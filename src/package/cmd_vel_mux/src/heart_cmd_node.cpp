/*
 * @Author: hambin
 * @Date: 2023-03-10 13:40:12
 * @LastEditors: Please set LastEditors
 * @FilePath: src/cmd_vel_mux.cpp
 * @Description: can to joy message
 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>

static void Handler(int sig)
{
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_mux");
    ros::NodeHandle nh;
    signal(SIGINT, Handler);
    ros::Publisher heart_pub = nh.advertise<geometry_msgs::Twist>("heart_cmd_vel", 1);
    while(ros::ok())
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x =0;
        cmd.angular.z=0;
        heart_pub.publish(cmd);
        ros::Rate(20).sleep();
    }

    return 0;
}
