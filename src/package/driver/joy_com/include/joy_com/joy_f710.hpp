
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef JOY_F710_HPP_
#define JOY_F710_HPP_

#include <iostream>
#include <string>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <boost/thread/pthread/shared_mutex.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <robot_state_msgs/robot_state.h>
#include <robot_state_msgs/data_to_stm32.h>
#include <robot_state_msgs/charge_action.h>
#include <robot_state_msgs/putter_action.h>
#include <robot_state_msgs/running_action.h>

namespace joy_com {

class Joyf710  {

public:
  Joyf710(ros::NodeHandle nh,ros::NodeHandle pnh);
  virtual ~Joyf710();

private:
  ros::Publisher  pubRobotCmd;
  ros::Subscriber subJoyData;
  ros::NodeHandle nhandle;
  ros::NodeHandle phandle;

  ros::Timer pubCmdTimer;
  boost::shared_mutex mutexPubCmd;
  robot_state_msgs::data_to_stm32 cmd_data;

  void JoyDataCallBack(const sensor_msgs::JoyConstPtr data);
  void JoySend(const ros::TimerEvent& event);
  int IsJoySend(const sensor_msgs::JoyConstPtr data);
};

}
#endif
