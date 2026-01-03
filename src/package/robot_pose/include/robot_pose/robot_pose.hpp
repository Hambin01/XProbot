
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ROBOT_POSE_HPP_
#define ROBOT_POSE_HPP_

#include <iostream>
#include <string>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <boost/thread/pthread/shared_mutex.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <robot_state_msgs/robot_state.h>
#include <robot_state_msgs/data_to_stm32.h>
#include <robot_state_msgs/charge_action.h>
#include <robot_state_msgs/putter_action.h>
#include <robot_state_msgs/running_action.h>

#include "../robot_pose/robot_kalman.hpp"

namespace robot_pose {

class RobotPose  {
#ifdef Work_Path
  std::string workspace_path = Work_Path;
#else
#error Work_Path not define in CMakelists.txt.
#endif
public:
  RobotPose(ros::NodeHandle nh,ros::NodeHandle pnh);
  virtual ~RobotPose();

private:
  ros::Publisher  pubRobotPose;
  ros::Publisher  pubRobotPoseInit;
  ros::NodeHandle nhandle;
  ros::NodeHandle phandle;
  std::thread poseSendThread;
  tf::TransformListener tfListener;
  RobotKalman  robotKalman;
  bool firstReceived = false;
  bool isRunning = false;
  nav_msgs::Odometry robotPose;
  ros::Timer  initTimer;
  boost::shared_mutex mutexInit;

  void SaveRobotPose(const ros::TimerEvent &event);
  void SendRobotPose(void);
  void RobotPoseSend(void);
  bool TransformFrame(nav_msgs::Odometry &s_pose,std::string goal_frame);
  nav_msgs::Odometry TransformToOdom(tf::Stamped<tf::Pose> &pose,
                                                std::string frame,
                                                std::string child_frame);
  double GetYawFromOrientation(geometry_msgs::Quaternion &q)
  {
    tf::Quaternion quat;
    tf::quaternionMsgToTF(q, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
  }

};

}
#endif

