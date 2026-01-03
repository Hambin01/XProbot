#include "../include/task_deal/task_deal.hpp"

namespace task_deal {

TaskDeal::TaskDeal(ros::NodeHandle nh, ros::NodeHandle pnh)
{
  /* read params */
  nhandle = nh;
  phandle = pnh;
  subRobotKeepTaskSend = nhandle.subscribe("/keep_task_send", 1, &TaskDeal::KeepTaskDealCallBack, this);
  subRobotOneTaskSend = nhandle.subscribe("/one_task_send", 1, &TaskDeal::OneTaskDealCallBack, this);
  //  pTask = popen("roslaunch cartographer_ros mapbuild.launch","r");
  taskName.push_back("roslaunch cartographer_ros mapbuild.launch");
  taskName.push_back("roslaunch cartographer_ros my_indoor_robot_backpack_2d_localization.launch");
  taskName.push_back("rosservice call /finish_trajectory  0");
  taskName.push_back("rosservice call /write_state \"{filename: '/home/robot/my.pbstream'}\"");
  taskName.push_back("rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=/home/robot/my -pbstream_filename=/home/robot/my.pbstream -resolution=0.05");
  ROS_INFO("task_deal_node start ok!");
}

TaskDeal::~TaskDeal()
{
  TaskEnd(pKeepTask);
  TaskEnd(pOneTask);
}

void TaskDeal::KeepTaskDealCallBack(const robot_task_msgs::robot_keep_task_sendConstPtr data)
{
  KEEP_TASK_TYPE curTask = (KEEP_TASK_TYPE)data->type;

  switch (curTask) {
  case MAP_BUILD:
    TaskExe(pKeepTask,taskName[MAP_BUILD].c_str(),"r");
    break;

  case LOCATION_START:
    TaskExe(pKeepTask,taskName[LOCATION_START].c_str(),"r");
    break;

  default:
    break;
  }

}

void TaskDeal::OneTaskDealCallBack(const robot_task_msgs::robot_one_task_sendConstPtr data)
{
  ONE_TASK_TYPE curTask = (ONE_TASK_TYPE)data->type;

  switch (curTask) {
  case MAP_STOP_BUILD:
    TaskExe(pKeepTask,taskStopName.c_str(),"r");
    break;

  case LOCATION_STOP:
    TaskExe(pKeepTask,taskStopName.c_str(),"r");
    break;

  case MAP_SAVE:
    if(TaskExe(pOneTask,taskName[MAP_SAVE].c_str(),"r")){
      TaskEnd(pOneTask);
      if(TaskExe(pOneTask,taskName[MAP_SAVE+1].c_str(),"r")){
        TaskEnd(pOneTask);
      }
    }
    break;

  case TO_MAP:
    if(TaskExe(pOneTask,taskName[MAP_SAVE+2].c_str(),"r")){
       TaskEnd(pOneTask);
  }
    break;

  default:
    break;
  }

}

bool TaskDeal::TaskExe(FILE *pTask,std::string taskName,const char *type)
{
  if(pTask !=NULL){
    ROS_WARN("Cant execute task,please stop first!");
    return false;
  }

  pTask = popen(taskName.c_str(),type);
  if(NULL == pTask){
    ROS_ERROR("%s execute fail!");
    return false;
  }
  return true;
}

bool TaskDeal::TaskEnd(FILE *pTask)
{
  if(pTask ==NULL){
    ROS_WARN("task ok!");
    return true;
  }

  int res = pclose(pTask);
  if(res == -1) {
    ROS_ERROR("stop task error!");
    return false;
  }else
    pTask =NULL;
  return true;
}

}
