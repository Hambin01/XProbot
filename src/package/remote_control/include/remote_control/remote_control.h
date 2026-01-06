
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int8.h>
#include <robot_state_msgs/robot_state.h>
#include <robot_state_msgs/data_to_stm32.h>
#include <robot_state_msgs/charge_action.h>
#include <robot_state_msgs/cross_action.h>
#include <robot_state_msgs/putter_action.h>
#include <robot_state_msgs/running_action.h>
#include <boost/thread/pthread/shared_mutex.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <jsoncpp/json/json.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <robot_task_msgs/robot_goal.h>
#include <robot_task_msgs/robot_navigation_cmd.h>
#include <robot_task_msgs/robot_goal_list.h>

#endif

namespace remote_control
{

  enum KEEP_TASK_TYPE
  {
    MAP_BUILD = 0,
    LOCATION_START,
  };

  enum ONE_TASK_TYPE
  {
    MAP_STOP_BUILD,
    LOCATION_STOP,
    MAP_SAVE,
    TO_MAP,
  };

  typedef enum RobotActionFlag
  {
    RobotIdle = 0,
    RobotRadiusRunning,
    RobotAngleRotating,
    RobotLinkRunning,
    RobotIniting,
    RobotCharging,
    RobotPausing,
    RobotFaulting,
    RobotSleeping,
    RobotAdjusting,
    RobotDischarging
  } RobotActionFlag;

  typedef enum RemoteCmd
  {
    NoCmd = 0,
    AGVInit = 1,
    CorssUpLeft = 2,
    CorssUpRight = 3,
    CorssDownLeft = 4,
    CorssDownRight = 5,
    PutterSet = 6,
    SetPathNode = 7,
    StartNavigation = 8,
    StopNavigation = 9,
    InsertNode = 10,
    ConnectStart = 11,
    DeleteStart = 12,
    ImportNodes = 13,
    SaveNodes = 14,
    SetDir = 15,
    SetCharge = 16,
    StartCharge = 17,
    StopCharge = 18,
    DeleteNode = 19,
    DebugLaser = 20

  } RemoteCmd;

  class RemoteControl
  {

  public:
    RemoteControl(ros::NodeHandle nh);

  protected:
    ros::NodeHandle nh_;

    ros::Publisher pubRobotCmd;
    ros::Publisher pubGoalSend;
    ros::Publisher pubNavigationStart;
    ros::Publisher pubNodeList;
    ros::Publisher pubGoalList;
    ros::Publisher pubDebugLaser;

    bool robotChargeFlag = false;
    ros::Subscriber subRobotPose;
    ros::Subscriber subClickPoint;
    ros::Subscriber subRemoteCmd;

    std::string node_file;

    boost::shared_mutex mutexReadPose;
    robot_state_msgs::data_to_stm32 cmd;
    geometry_msgs::PoseStamped robotPose;
    geometry_msgs::PoseStamped goal;
    std::map<std::string, robot_task_msgs::robot_goal> nodeList;
    std::vector<std::pair<std::string, std::string>> line_lists;

    Json::Value convert_pose_to_json(geometry_msgs::Pose pose);
    geometry_msgs::Pose convert_pose_to_json(Json::Value &list);
    visualization_msgs::Marker convert_node_to_mark(int id,
                                                    std::string name,
                                                    geometry_msgs::Pose &pose, int type);
    bool connectFlag = false;
    bool deleteFlag = false;
    bool firstConnectFlag = false;
    bool firstDeleteFlag = false;
    void connect_two_points(visualization_msgs::Marker &line_list, geometry_msgs::Pose &p1, geometry_msgs::Pose &p2);
    void pub_node_list(void);
    void RobotPoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose);
    void ClickPointCallBack(const geometry_msgs::PointStampedConstPtr point);
    void RemoteCmdCallBack(const std_msgs::HeaderConstPtr cmd_control);
    void disconnect_two_points(std::string first, std::string second);
    void import_nodes(std::string fileName);

    void on_setGoal_clicked(std_msgs::Header &cmd_remote);
    void on_startNavigation_clicked(std_msgs::Header &cmd_remote);
    void on_insertNode_clicked(std_msgs::Header &cmd_remote);
    void on_nodeClear_clicked(std_msgs::Header &cmd_remote);
    void on_importNode_clicked(std_msgs::Header &cmd_remote);
    void on_deleteNode_clicked(std_msgs::Header &cmd_remote);
    void on_saveNode_clicked(std_msgs::Header &cmd_remote);
    void on_stopNavigation_clicked(std_msgs::Header &cmd_remote);

    void on_cross_clicked(std_msgs::Header &cmd_remote);
    void on_crossSpeedSet_clicked(std_msgs::Header &cmd_remote);
    void on_startCharge_clicked(std_msgs::Header &cmd_remote);
    void on_clearPath_clicked(std_msgs::Header &cmd_remote);
    void on_stopCharge_clicked(std_msgs::Header &cmd_remote);
    void on_setCharge_clicked(std_msgs::Header &cmd_remote);
    void on_agvInit_clicked(std_msgs::Header &cmd_remote);
    void on_ConnectSet_clicked(std_msgs::Header &cmd_remote);
    void on_deleteSet_clicked(std_msgs::Header &cmd_remote);
    void on_setDirection_clicked(std_msgs::Header &cmd_remote);
    void on_debugLaser_clicked(std_msgs::Header &cmd_remote);
  };

}

#endif