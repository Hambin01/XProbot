/*
 * @Author: your name
 * @Date: 2021-07-20 11:58:45
 * @LastEditTime: 2021-07-30 17:43:39
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edi
 * @FilePath: /xm-autosystem/src/package/application/robot_pose/include/robot_pose/robot_pose.hpp
 */

/*****************************************************************************
** Ifdefs
*****************************************************************************/

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
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PCLHeader.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <robot_state_msgs/data_to_stm32.h>

namespace laser_avoidance {


struct AvoidDirection
{
  float x_min;
  float x_max;
  float y_min;
  float y_max;
  int   laser_count;
};

class LaserAvoidance  {
#ifdef Work_Path
  std::string workspace_path = Work_Path;
#else
#error Work_Path not define in CMakelists.txt.
#endif
public:
  LaserAvoidance(ros::NodeHandle nh,ros::NodeHandle pnh);
  virtual ~LaserAvoidance();

private:
  ros::Publisher pub_laser_filter;
  ros::Publisher pub_avoidance_msg;
  ros::Publisher pub_adjust_req;
  ros::Publisher pub_adjust_motion;
  ros::Publisher pub_debug_msg;

  ros::Subscriber sub_avoidance_msg;
  ros::Subscriber sub_laser_msgs;
  ros::Subscriber sub_adjust_cmd;
  ros::Subscriber sub_debug_cmd;

  ros::NodeHandle nhandle;
  ros::NodeHandle phandle;

  tf::TransformListener tfListener;
  laser_geometry::LaserProjection projector;

  AvoidDirection move_dev_avoidance;
  AvoidDirection move_stop_avoidance;
  AvoidDirection rotate_avoidance;
  AvoidDirection trans_avoidance;
  AvoidDirection robot_polygon;
  float laser_resultion;
  float laser_offset_angle;
  int avoidance_enable = 1;
  float robot_width,robot_length,robot_center_to_front;

  std::string http_interance_;
  std::string robot_id_ = "1";
  int max_fitter =900;
  double offset =0;
  float  min_range = 0.1;
  float  max_range = 1.0;

  bool  adjust_flag = false;
  bool  adjust_debug = false;
  
  boost::shared_mutex mutexInit;

  void LoadAvoidanceParam(std::string dir);
  void LaserHandler(const sensor_msgs::LaserScanConstPtr &scan);
  void CloudToLaser(const sensor_msgs::PointCloud2& msg,
                    sensor_msgs::LaserScan &laser);
  void LaserFilter(const sensor_msgs::LaserScan& msg);
  void AvoidanceDeal(const sensor_msgs::PointCloud2& cloud,
         std_msgs::Int8 &avoid_status);
  void AvoidanceSetHandler(const std_msgs::Int8ConstPtr &msg);
  void AdjustSetHandler(const std_msgs::Int8ConstPtr &msg);
  bool IsIntersects(AvoidDirection& direction,float x,float y);
  void DebugSetHandler(const std_msgs::Int8ConstPtr &msg);


};

}
#endif

