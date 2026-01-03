
#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <rviz/panel.h>

#include <nav_msgs/Odometry.h>
#include <robot_state_msgs/robot_state.h>
#include <robot_state_msgs/data_to_stm32.h>
#include <robot_state_msgs/charge_action.h>
#include <robot_state_msgs/cross_action.h>
#include <robot_state_msgs/putter_action.h>
#include <robot_state_msgs/running_action.h>
#include <robot_task_msgs/robot_keep_task_send.h>
#include <robot_task_msgs/robot_one_task_send.h>
#include "../ui/ui_image_viewer.h"
#include <boost/thread/pthread/shared_mutex.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <json/json.h>
#include <QFileDialog>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <robot_task_msgs/robot_goal.h>
#include <robot_task_msgs/robot_navigation_cmd.h>
#include <robot_task_msgs/robot_goal_list.h>

#endif

namespace robot_control
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

  class RobotControl : public rviz::Panel
  {

    Q_OBJECT
  public:
#ifdef Work_Path
    QString workspace_path = Work_Path;
#else
#error Work_Path not define in CMakelists.txt.
#endif

    RobotControl(QWidget *parent = 0);

    virtual void load(const rviz::Config &config);
    virtual void save(rviz::Config config) const;

  protected:
    Ui::image_viewer_form ui_;
    ros::NodeHandle nh_;

    ros::Publisher pubKeepTask;
    ros::Publisher pubOneTask;
    ros::Publisher pubRobotCmd;
    ros::Publisher pubGoalSend;
    ros::Publisher pubNavigationStart;
    ros::Publisher pubNodeList;
    ros::Publisher pubGoalList;

    bool robotMoveFlag = false;
    bool robotChargeFlag = false;
    int move_count = 0;
    ros::Timer moveTimer;
    ros::Subscriber subRobotPose;
    ros::Subscriber subRobotState;
    ros::Subscriber subClickPoint;

    boost::shared_mutex mutexReadPose;
    robot_state_msgs::robot_state robotState;
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
    void CmdSet(const ros::TimerEvent &event);
    void RobotPoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose);
    void StateCallBack(const robot_state_msgs::robot_stateConstPtr data);
    void ClickPointCallBack(const geometry_msgs::PointStampedConstPtr point);
    void disconnect_two_points(std::string first, std::string second);
    void import_nodes(void);
    void import_edges(void);
    void import_nodes(std::string fileName);

  protected Q_SLOTS:
    //  void on_MapBuildStart_clicked();
    //  void on_SaveMap_clicked();
    //  void on_MapBUildStop_clicked();
    //  void on_LocationStart_clicked();
    //  void on_LocationStop_clicked();

    void on_robot_go_pressed();
    void on_robot_back_pressed();
    void on_robot_right_pressed();
    void on_robot_left_pressed();
    void on_robot_go_released();
    void on_robot_back_released();
    void on_robot_left_released();
    void on_robot_right_released();

    void on_setGoal_clicked();
    void on_startNavigation_clicked();
    void on_insertNode_clicked();
    void on_nodeClear_clicked();
    void on_importNode_clicked();
    void on_saveNode_clicked();
    void on_stopNavigation_clicked();
    void on_comboBox_currentIndexChanged(const QString &arg1);

    //  void on_putterSet_clicked();
    //  void on_putterInit_clicked();
    void on_crossUp_clicked();
    void on_crossDown_clicked();
    void on_crossSpeedSet_clicked();
    void on_startCharge_clicked();
    void on_clearPath_clicked();
    void on_stopCharge_clicked();
    void on_setCharge_clicked();
    void on_agvInit_clicked();
    void on_ConnectSet_clicked();
    void on_deleteSet_clicked();
    void on_setDirection_clicked();
  };

}

#endif