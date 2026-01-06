/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-03-13 14:30:58
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2023-03-22 09:43:01
 * @FilePath: /can_package/src/pose_init/include/pose_init.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#pragma once
#ifndef _POSE_INIT_H_
#define _POSE_INIT_H_


#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
 
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace pose_init
{

class PoseInit
{


public:
    PoseInit(ros::NodeHandle &nhandle);
    ~PoseInit(){}

    bool ImportParams(std::string yaml_file);
    void PoseCallback(const geometry_msgs::PoseConstPtr msg);

    void AmclPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr msg);

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_pose;
    ros::Subscriber sub_amcl_pose;
    ros::Publisher pub_init_pose;
    ros::Time  write_time;
    std::string yaml_file;
    float save_period = 1.0;
    bool use_amcl_pose = false;
};
}

#endif
