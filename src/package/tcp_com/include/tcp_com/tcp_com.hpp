
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef TCP_COM_HPP_
#define TCP_COM_HPP_

#include <iostream>
#include <string>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <boost/thread/pthread/shared_mutex.hpp>

#include<errno.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<netinet/in.h>
#include<unistd.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <robot_state_msgs/robot_state.h>
#include <robot_state_msgs/data_to_stm32.h>
#include <robot_state_msgs/charge_action.h>
#include <robot_state_msgs/putter_action.h>
#include <robot_state_msgs/running_action.h>
#include <robot_task_msgs/robot_goal.h>
#include <../include/tcp_com/tcp_client.hpp>
#include <../include/tcp_com/tcp_service.hpp>

namespace tcp_com {

class TcpCom  {

public:
  TcpCom(ros::NodeHandle nh,ros::NodeHandle pnh);
  virtual ~TcpCom(){}

};

}
#endif
