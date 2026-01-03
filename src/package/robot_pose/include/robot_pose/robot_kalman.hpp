#ifndef ROBOT_KALMAN_HPP
#define ROBOT_KALMAN_HPP

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <nav_msgs/Odometry.h>

namespace robot_pose
{
typedef struct KalmanState
{
  cv::KalmanFilter kf_;
  cv::Mat state_;
  cv::Mat predict_state_;
  cv::Mat measure_;
}KalmanState;

class RobotKalman
{
public:
  void init(nav_msgs::Odometry &pose);
  void update(nav_msgs::Odometry &pose);
  void predict();
  bool outofbound();
  nav_msgs::Odometry get_state();

  geometry_msgs::Pose  robotPose ;

private:
  int type_ = CV_32F;
  int stateSize_ = 6;
  int measSize_ = 2;
  int contrSize_ = 0;
  int path_size = 100;

  KalmanState kf_;
  ros::Time last;

  void set_kalman_param(float dt);

};

} // namespace sort

#endif
