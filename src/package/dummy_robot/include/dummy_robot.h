/*
 * @Author: Hambin.Lu
 * @Description: ##
 */

#pragma once
#ifndef _dummy_robot_H_
#define _dummy_robot_H_


#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <sstream>
#include <yaml-cpp/yaml.h>

namespace dummy_robot
{

class DummyRobot
{


public:
    DummyRobot(ros::NodeHandle &nhandle);
    ~DummyRobot(){}

    

private:
    ros::NodeHandle nh;
    //ros::Publisher 
    //ros::Subscriber 

};
}

#endif
