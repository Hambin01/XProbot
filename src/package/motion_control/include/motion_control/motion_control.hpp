
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef NAVIGATION_LINE_HPP_
#define NAVIGATION_LINE_HPP_

#include <iostream>
#include <string>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/pthread/shared_mutex.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d.h>
// #include <nav_core/base_global_planner.h>
// #include <nav_core/base_local_planner.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include "kalman.hpp"
#include <std_msgs/Int8.h>

#include <motion_control/motion_control_cfgConfig.h>
#include "dynamic_reconfigure/server.h"
#include <actionlib/server/simple_action_server.h>
#include <motion_control/robotNavAction.h>
#include <motion_control/robotNavResult.h>
#include <motion_control/robotNavFeedback.h>
#include <motion_control/robotNavGoal.h>
#include <robot_state_msgs/robot_state.h>
#include <robot_state_msgs/data_to_stm32.h>

namespace navigation
{

  class Navigation
  {

    enum Adjust_Type
    {
      STOP = 0,
      ROTATE,
      GO_LINE,
      Trans,
      ROTATE_GOAL,
      RUN_ERROR,
      RUN_ADJUST
    };

  public:
    Navigation(ros::NodeHandle nh, ros::NodeHandle pnh);
    virtual ~Navigation();

  private:
    ros::NodeHandle nhandle;
    ros::NodeHandle phandle;
    ros::Subscriber subRobotState;
    ros::Subscriber navigation_cancel_subscriber_;
    ros::Subscriber adjust_req_subscriber_;
    ros::Subscriber robot_pose_subscriber_;

    ros::Publisher navigation_speed_publisher_;
    ros::Publisher adjust_cmd_publisher_;
    ros::Publisher cmd_vel_publisher_;

    pid::PidClass pidCal;
    float kt;
    float acc;
    float stop_distance;
    float max_speed;
    float min_speed;
    float stop_speed;
    float rotate_speed;
    float stop_radio;
    float max_pid;
    float dis_radio;
    float rotate_stop;
    float dev_angle;
    float stop_angle;
    float localization_rate;
    float goal_distance_threshold;
    float goal_angle_threshold;
    float adjust_distance;

    bool maxSpeedFlag = false;
    bool navigation_pause;
    int dir_type = 0;
    int run_adjust = 0;
    bool use_run_adjust = true;

    robot_state_msgs::robot_state robotState;
    Adjust_Type action_type = STOP;
    geometry_msgs::PoseStamped goalPose;
    geometry_msgs::PoseStamped startPose;

    boost::shared_mutex mutexPose;
    geometry_msgs::PoseStamped robot_pose_;
    tf::TransformListener tf_listen_;
    robot_state_msgs::data_to_stm32 cmd;

    typedef actionlib::SimpleActionServer<motion_control::robotNavAction> navigation_server;
    navigation_server nav_server;
    motion_control::robotNavGoal navGoal;
    motion_control::robotNavResult navResult;
    motion_control::robotNavFeedback navFeedback;

    dynamic_reconfigure::Server<motion_control::motion_control_cfgConfig> server;
    dynamic_reconfigure::Server<motion_control::motion_control_cfgConfig>::CallbackType f;

    void CfgCallback(motion_control::motion_control_cfgConfig &cfg, uint32_t level);
    void AdjustReqCallback(const std_msgs::Int8ConstPtr &msg);
    void StateCallBack(const robot_state_msgs::robot_stateConstPtr &msg);
    void NavigationCancelCallBack(const actionlib_msgs::GoalID &msg);
    void RobotPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void Execute(const motion_control::robotNavGoalConstPtr &goal);
    bool RotateAdjust(double angle, int dir);
    bool TransAdjust(double distance);
    bool GoLineAdjust(double angle, double t_dis, double t_angle, double distance);
    float GetAngelOfTwoVector(geometry_msgs::PoseStamped &pt1,
                              geometry_msgs::PoseStamped &pt2,
                              geometry_msgs::PoseStamped &c);
    double GetDistanceToLine(geometry_msgs::PoseStamped &pt1,
                             geometry_msgs::PoseStamped &pt2,
                             geometry_msgs::PoseStamped &c);

    double NeedAcc(double setSpeed, double curSpeed);
    void CalcAdjust(double &abs_angle,
                    double &dis,
                    double &goal_angle,
                    double &t_dis,
                    double &t_angle);
    void ResetParam(void);
    void ResetSpeed(void);
    void SetSpeed(float speed, float radius, float angle);
  };

}
#endif
