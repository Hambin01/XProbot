
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/joy_com/joy_f710.hpp"

namespace joy_com {

Joyf710::Joyf710(ros::NodeHandle nh, ros::NodeHandle pnh)
{
  /* read params */
  nhandle = nh;
  phandle = pnh;
  pubRobotCmd = nhandle.advertise<robot_state_msgs::data_to_stm32>("data_to_stm32", 2);
  subJoyData = nhandle.subscribe("/joy", 1, &Joyf710::JoyDataCallBack, this);
  pubCmdTimer = phandle.createTimer(ros::Duration(0.1),&Joyf710::JoySend, this);

  ROS_INFO("joy node start ok !");
}

Joyf710::~Joyf710()
{

}

void Joyf710::JoyDataCallBack(const sensor_msgs::JoyConstPtr data)
{
  robot_state_msgs::data_to_stm32 cmd;
  boost::unique_lock<boost::shared_mutex> lockDeal(mutexPubCmd);
  int cmd_index = IsJoySend(data);

  switch (cmd_index) {
  case 0x02:
    cmd.task_type = cmd_index;
    cmd.running.type = 1;
    cmd.running.speed = data->axes[4]*0.4;
    cmd.running.radius = -data->axes[3]*0.5;
    break;

  case 0x04:
    cmd.task_type = cmd_index;
    break;

  case 0x0D:
    cmd.task_type = cmd_index;
    if(data->buttons[0] == 1)
      cmd.cross.type = 1;
    if(data->buttons[1] == 1)
      cmd.cross.type = 2;
    if(data->buttons[2] == 1)
      cmd.cross.type = 3;
    if(data->buttons[3] == 1)
      cmd.cross.type = 4;
    break;

  default:
    break;
  }
  cmd_data = cmd;
}

void Joyf710::JoySend(const ros::TimerEvent& event)
{
  if(cmd_data.task_type!=0){
    boost::unique_lock<boost::shared_mutex> lockDeal(mutexPubCmd);
    cmd_data.header.stamp = ros::Time::now();
    pubRobotCmd.publish(cmd_data);
  }
}

int Joyf710::IsJoySend(const sensor_msgs::JoyConstPtr data)
{
  float buttons_total;

  for(int i=0;i<12;i++)
    buttons_total+=data->buttons[i];

  if(buttons_total!=0){
    if(buttons_total ==1 && data->buttons[7]==1)
      return 0x04;
    if(buttons_total ==1 && (data->buttons[0]|data->buttons[1]|data->buttons[2]|data->buttons[3] == 1))
      return 0x0D;
    return 0;
  }

  if(data->axes[3] !=0 || data->axes[4] !=0)
    return 0x02;

  return 0;
}

}
