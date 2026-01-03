
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/robot_pose/robot_kalman.hpp"
#include <algorithm>

namespace robot_pose {

void RobotKalman::init(nav_msgs::Odometry &pose)
{

  kf_.kf_ = cv::KalmanFilter(stateSize_, measSize_, contrSize_, type_);
  kf_.state_ = cv::Mat(stateSize_, 1, type_);  // [x,y,vx,vy,ax,ay]
  kf_.measure_ = cv::Mat(measSize_, 1, type_); // [x,y]

  // state
  kf_.state_.at<float>(0) = pose.pose.pose.position.x;
  kf_.state_.at<float>(1) = pose.pose.pose.position.y;
  kf_.state_.at<float>(2) = 0;
  kf_.state_.at<float>(3) = 0;
  kf_.state_.at<float>(4) = 0;
  kf_.state_.at<float>(5) = 0;

  // measure
  kf_.measure_.setTo(cv::Scalar(0));

  // State Transition Matrix A
  // Note: set dT at each processing step!
  // [ 1 0 dt  0 0.5*dt^2     0]
  // [ 0 1 0  dt    0     0.5*dt^2]
  // [ 0 0 1  0     dt        0 ]
  // [ 0 0 0  1     0         dt]
  // [ 0 0 0  0     1         0 ]
  // [ 0 0 0  0     0         1 ]
  cv::setIdentity(kf_.kf_.transitionMatrix);
  set_kalman_param(0.14f);

  // Measure Matrix H
  // [ 1 0 0 0 0 0]
  // [ 0 1 0 0 0 0]
  kf_.kf_.measurementMatrix = cv::Mat::zeros(measSize_, stateSize_, type_);
  kf_.kf_.measurementMatrix.at<float>(0) = 1.0f;
  kf_.kf_.measurementMatrix.at<float>(7) = 1.0f;

  // Measures Noise Covariance Matrix R
  // [1 0 ]
  // [0 1 ]
  cv::setIdentity(kf_.kf_.measurementNoiseCov, cv::Scalar(1.0f));
  kf_.kf_.measurementNoiseCov.at<float>(0) = 0.02f;
  kf_.kf_.measurementNoiseCov.at<float>(3) = 0.02f;

  // Posteriori error estimate covariance matrix (P(k))
  kf_.kf_.errorCovPost.at<float>(0) = 1.0f;
  kf_.kf_.errorCovPost.at<float>(7) = 1.0f;
  kf_.kf_.errorCovPost.at<float>(14) = 10000.0f;
  kf_.kf_.errorCovPost.at<float>(21) = 10000.0f;
  kf_.kf_.errorCovPost.at<float>(28) = 10000.0f;
  kf_.kf_.errorCovPost.at<float>(35) = 10000.0f;

  // Process Noise Covariance Matrix Q
  kf_.kf_.processNoiseCov.at<float>(0) = 0.1f;
  kf_.kf_.processNoiseCov.at<float>(7) = 0.1f;
  kf_.kf_.processNoiseCov.at<float>(14) = 0.01f;
  kf_.kf_.processNoiseCov.at<float>(21) = 0.01f;
  kf_.kf_.processNoiseCov.at<float>(28) = 0.001f;
  kf_.kf_.processNoiseCov.at<float>(35) = 0.001f;

  kf_.kf_.statePost = kf_.state_;
  robotPose.position.x = pose.pose.pose.position.x;
  robotPose.position.y = pose.pose.pose.position.y;
  last = ros::Time::now();
}

void RobotKalman::set_kalman_param(float dt)
{
  kf_.kf_.transitionMatrix.at<float>(2) = dt;
  kf_.kf_.transitionMatrix.at<float>(4) = 0.5*dt*dt;
  kf_.kf_.transitionMatrix.at<float>(9) = dt;
  kf_.kf_.transitionMatrix.at<float>(11) = 0.5*dt*dt;
  kf_.kf_.transitionMatrix.at<float>(16) = dt;
  kf_.kf_.transitionMatrix.at<float>(23) = dt;
}

void RobotKalman::update(nav_msgs::Odometry &pose)
{
  kf_.measure_.at<float>(0) = pose.pose.pose.position.x;
  kf_.measure_.at<float>(1) = pose.pose.pose.position.y;
  kf_.kf_.correct(kf_.measure_);
  robotPose.position.x = pose.pose.pose.position.x;
  robotPose.position.y = pose.pose.pose.position.y;
}

void RobotKalman::predict()
{
  float dt = (ros::Time::now()-last).toSec();
  set_kalman_param(dt);

  if (outofbound())
      return;
  kf_.state_ = kf_.kf_.predict();
  kf_.predict_state_ = kf_.state_.clone();
  last = ros::Time::now();
}

nav_msgs::Odometry RobotKalman::get_state()
{
  nav_msgs::Odometry pose;
  robotPose.position.x = kf_.state_.at<float>(0);
  robotPose.position.y = kf_.state_.at<float>(1);
  robotPose.position.z  = kf_.state_.at<float>(2);
  robotPose.orientation.x  = kf_.state_.at<float>(3);
  robotPose.orientation.y = kf_.state_.at<float>(4);
  robotPose.orientation.z = kf_.state_.at<float>(5);

  pose.pose.pose.position.x = robotPose.position.x;
  pose.pose.pose.position.y = robotPose.position.y;

  return pose;
}

bool RobotKalman::outofbound()
{
  return false;
}

}
