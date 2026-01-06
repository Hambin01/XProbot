
#ifndef LASER_AVOIDANCE_HPP_
#define LASER_AVOIDANCE_HPP_

#include <iostream>
#include <string>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <memory>
#include <boost/thread/pthread/shared_mutex.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <robot_state_msgs/data_to_stm32.h>

namespace laser_avoidance
{

  class LaserAvoidance
  {

  public:
    LaserAvoidance(ros::NodeHandle nh, ros::NodeHandle pnh);
    virtual ~LaserAvoidance();

  private:
    ros::Publisher pub_laser_filter;
    ros::Publisher pub_adjust_req;
    ros::Publisher pub_adjust_motion;
    ros::Publisher pub_adjust_motion_cmd_vel;

    ros::Subscriber sub_laser_msgs;
    ros::Subscriber sub_adjust_cmd;
    ros::Subscriber sub_debug_cmd;

    ros::NodeHandle nhandle;
    ros::NodeHandle phandle;

    tf::TransformListener tfListener;
    laser_geometry::LaserProjection projector;

    float laser_resultion;
    float laser_offset_angle;

    int max_fitter = 900;
    double offset = 0;
    double min_range = 0.1;
    double max_range = 1.0;

    bool adjust_flag = false;
    bool adjust_debug = false;

    void LaserHandler(const sensor_msgs::LaserScanConstPtr &scan);
    void LaserFilter(const sensor_msgs::LaserScan &msg);
    void AdjustSetHandler(const std_msgs::Int8ConstPtr &msg);
    void DebugSetHandler(const std_msgs::Int8ConstPtr &msg);
  };

}
#endif
