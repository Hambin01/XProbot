
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef NAVIGATION_PLAN_HPP_
#define NAVIGATION_PLAN_HPP_

#include <iostream>
#include <string>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <boost/thread/pthread/shared_mutex.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <robot_state_msgs/robot_state.h>
#include <robot_state_msgs/data_to_stm32.h>
#include <robot_state_msgs/charge_action.h>
#include <robot_state_msgs/putter_action.h>
#include <robot_state_msgs/running_action.h>
#include <robot_task_msgs/robot_goal.h>
#include <robot_task_msgs/robot_navigation_cmd.h>
#include <robot_task_msgs/robot_goal_list.h>
#include <robot_task_msgs/robot_task_req.h>

#include "path_plan.hpp"
#include <visualization_msgs/Marker.h>
#include <motion_control/robotNavAction.h>
#include <actionlib/client/simple_action_client.h>
#include <jsoncpp/json/json.h>
#include <geometry_msgs/Twist.h>

namespace navigation
{

  typedef actionlib::SimpleActionClient<motion_control::robotNavAction> NavigationClient;

  class Navigation
  {

    enum Adjust_Type
    {
      STOP = 0,
      MOVE,
      RUN_ERROR,
      PUTTER_ACTION,
      CROSS_ACTION,
      CHARGE_ACTION
    };

    enum Action_Type
    {
      NO_ACTION = 0,
      PUTTER,
      CROSS
    };

    enum Req_Type
    {
      MOVE_SET = 1,
      CROSS_SET_UP,
      CROSS_SET_DOWN,
      STATE_GET,
      PUTTER_SET,
      CHARGE_SET,
      CHARGE_STOP_SET,
      ROBOT_YAW_GET,
      STOP_SET,
      LAST_NODE_GET,
      CROSS_STATE,
      ROBOT_GET,
      TASK_STATE_GET,
      TASK_STATE_GET2,
      CROSS_STATE2
    };

    typedef enum RobotActionFlag
    {
      RobotIdle = 0,
      RobotRadiusRunning,
      RobotAngleRotating,
      RobotIniting,
      RobotCharging,
      RobotPausing,
      RobotFaulting,
      RobotSleeping,
      RobotPuttering,
      RobotCrossing,
      RobotForcePutterReseting
    } RobotActionFlag;

  public:
    Navigation(ros::NodeHandle nh, ros::NodeHandle pnh);
    virtual ~Navigation();

  private:
    ros::Publisher pubRobotCmd;
    ros::Publisher pubTaskPoint;
    ros::Publisher pubTaskReq;
    ros::Publisher pubNavigationCancel;
    ros::Publisher pubCmdVel;
    ros::Subscriber subTaskPoint;
    ros::Subscriber subGoal;
    ros::Subscriber subRobotPose;
    ros::Subscriber subRobotState;
    ros::Subscriber subNodeList;
    ros::Subscriber subTaskCmd;
    ros::NodeHandle nhandle;
    ros::NodeHandle phandle;

    ros::Timer taskReqTimer;
    robot_state_msgs::data_to_stm32 cmd;
    geometry_msgs::Twist cmd_vel;
    boost::shared_mutex mutexControl;
    std::thread *controlThread;

    geometry_msgs::PoseStamped robotPose;
    robot_state_msgs::robot_state robotState;
    std::vector<std::string> taskPoint;
    std::string chargePoint = "none";
    robot_task_msgs::robot_goal goalNode;
    std::map<std::string, robot_task_msgs::robot_goal> nodeList;
    robot_task_msgs::robot_task_req taskReq;
    actionlib_msgs::GoalID cancel_goal;

    bool use_replay;
    // pwq
    bool have_cargo = true;
    bool is_load_cargo = true;
    int last_cross_type = 0;

    Adjust_Type action_type = STOP;
    NavigationClient navigation_client;
    PathPlan patPlan;

    int last_node;
    int now_node;
    int goal_node = 0;
    bool cross_flag;
    ros::Time task_start_time;
    bool back_flag = false;
    int task_state;
    std::string node_file;

    void GoalSetCall(robot_task_msgs::robot_goal goal);
    void GoalsSetCall(void);
    void NavigationSetCall(robot_task_msgs::robot_navigation_cmd cmd);
    void PoseCallBack(geometry_msgs::PoseStampedConstPtr pose);
    void StateCallBack(const robot_state_msgs::robot_stateConstPtr data);
    void NodeListCallBack(const robot_task_msgs::robot_goal_listConstPtr list);
    void TaskCmdCallBack(const robot_task_msgs::robot_task_reqConstPtr msg);
    geometry_msgs::Pose convert_pose_to_json(Json::Value &list);
    void import_nodes(std::string fileName);
    void InitParam(void);

    void MoveToGoal(void);
    bool TaskAction(void);
    bool MoveAction(void);
    bool StopChargeAction(void);
    bool ChargeAction(void);
    void TaskReq(const ros::TimerEvent &event);
    void SetSpeed(float speed, float radius, float angle);
  };

}
#endif