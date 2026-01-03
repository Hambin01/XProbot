
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef TASK_DEAL_HPP_
#define TASK_DEAL_HPP_

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <boost/thread/pthread/shared_mutex.hpp>
#include <robot_task_msgs/robot_keep_task_send.h>
#include <robot_task_msgs/robot_one_task_send.h>

#include <ros/ros.h>

namespace task_deal {

#ifdef Work_Path
  std::string workspace_path = Work_Path;
#else
#error Work_Path not define in CMakelists.txt.
#endif

enum KEEP_TASK_TYPE{
  MAP_BUILD=0,
  LOCATION_START,
};

enum ONE_TASK_TYPE{
  MAP_STOP_BUILD,
  LOCATION_STOP,
  MAP_SAVE,
  TO_MAP,
};

class TaskDeal  {

public:
  TaskDeal(ros::NodeHandle nh,ros::NodeHandle pnh);
  virtual ~TaskDeal();

private:
  ros::Publisher  pubRobotTaskDealReq;
  ros::Subscriber subRobotKeepTaskSend;
  ros::Subscriber subRobotOneTaskSend;
  ros::NodeHandle nhandle;
  ros::NodeHandle phandle;

  std::vector<std::string> taskName;
  std::string taskStopName = "bash "+workspace_path+"/stop_roslaunch.sh";

  FILE *pKeepTask = NULL;
  FILE *pOneTask = NULL;

  void KeepTaskDealCallBack(const robot_task_msgs::robot_keep_task_sendConstPtr data);
  void OneTaskDealCallBack(const robot_task_msgs::robot_one_task_sendConstPtr data);
  bool TaskExe(FILE *pTask, std::string taskName, const char *type);
  bool TaskEnd(FILE *pTask);

};

}
#endif
