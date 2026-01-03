/*
 * @Author: Hambin.Lu
 * @Description: ##
 */

#pragma once
#ifndef _video_publisher_H_
#define _video_publisher_H_


#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <sstream>
#include <yaml-cpp/yaml.h>

namespace video_publisher
{

class VideoPublisher
{


public:
    VideoPublisher(ros::NodeHandle &nhandle);
    ~VideoPublisher(){}

    

private:
    ros::NodeHandle nh;
    //ros::Publisher 
    //ros::Subscriber 

};
}

#endif
