#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <rviz_common/panel.hpp>
#include <QWidget>
#include "ui_image_viewer.h"

// ROS2 核心头文件
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>

// 消息头文件
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <robot_state_msgs/msg/data_to_stm32.hpp>
#include <robot_state_msgs/msg/robot_state.hpp>
#include <robot_task_msgs/msg/robot_goal.hpp>
#include <robot_task_msgs/msg/robot_goal_list.hpp>
#include <robot_task_msgs/msg/robot_keep_task_send.hpp>
#include <robot_task_msgs/msg/robot_one_task_send.hpp>
#include <robot_task_msgs/msg/robot_navigation_cmd.hpp>

// 其他依赖
#include <nlohmann/json.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <shared_mutex>  // C++17 共享互斥锁（核心替换）
#include <mutex> 

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

class RobotControl : public rviz_common::Panel
{
  Q_OBJECT  // 必须保留，且位于类声明第一行！

public:
  // 构造函数（显式声明）
  explicit RobotControl(QWidget *parent = nullptr);
  
  // 虚析构函数（必须声明+实现，否则 vtable 缺失）
  virtual ~RobotControl() override;
  void onInitialize() override;

  // 重写 RViz Panel 的虚函数（必须实现！）
  void load(const rviz_common::Config &config) override;
  void save(rviz_common::Config config) const override;

protected Q_SLOTS:
  // 所有 UI 槽函数声明（确保与 UI 控件信号匹配）
  void on_robot_go_pressed();
  void on_robot_go_released();
  void on_robot_back_pressed();
  void on_robot_back_released();
  void on_robot_left_pressed();
  void on_robot_left_released();
  void on_robot_right_pressed();
  void on_robot_right_released();
  void on_setGoal_clicked();
  void on_insertNode_clicked();
  void on_nodeClear_clicked();
  void on_importNode_clicked();
  void on_saveNode_clicked();
  void on_comboBox_currentIndexChanged(const QString &arg1);
  void on_stopNavigation_clicked();
  void on_startNavigation_clicked();
  void on_clearPath_clicked();
  void on_crossUp_clicked();
  void on_crossDown_clicked();
  void on_crossSpeedSet_clicked();
  void on_startCharge_clicked();
  void on_stopCharge_clicked();
  void on_setCharge_clicked();
  void on_agvInit_clicked();
  void on_ConnectSet_clicked();
  void on_deleteSet_clicked();
  void on_setDirection_clicked();

private:
  // ROS2 节点指针（核心！必须声明）
  rclcpp::Node::SharedPtr nh_;

  // UI 对象（与生成的 UI 头文件匹配）
  Ui::image_viewer_form ui_;

  // 发布者声明（确保类型匹配）
  rclcpp::Publisher<robot_task_msgs::msg::RobotKeepTaskSend>::SharedPtr pubKeepTask;
  rclcpp::Publisher<robot_task_msgs::msg::RobotOneTaskSend>::SharedPtr pubOneTask;
  rclcpp::Publisher<robot_state_msgs::msg::DataToStm32>::SharedPtr pubRobotCmd;
  rclcpp::Publisher<robot_task_msgs::msg::RobotGoal>::SharedPtr pubGoalSend;
  rclcpp::Publisher<robot_task_msgs::msg::RobotNavigationCmd>::SharedPtr pubNavigationStart;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubNodeList;
  rclcpp::Publisher<robot_task_msgs::msg::RobotGoalList>::SharedPtr pubGoalList;

  // 订阅者声明
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subRobotPose;
  rclcpp::Subscription<robot_state_msgs::msg::RobotState>::SharedPtr subRobotState;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subClickPoint;

  // 定时器声明
  rclcpp::TimerBase::SharedPtr moveTimer;

  // 其他成员变量（根据实际代码补充）
  nav_msgs::msg::Odometry robotPose;
  robot_state_msgs::msg::RobotState robotState;
  robot_state_msgs::msg::DataToStm32 cmd;
  std::shared_mutex mutexReadPose;
  bool robotMoveFlag = false;
  bool robotChargeFlag = false;
  int move_count = 0;
  bool connectFlag = false;
  bool firstConnectFlag = false;
  bool deleteFlag = false;
  bool firstDeleteFlag = false;

  // 节点列表相关（根据实际代码补充）
  std::map<std::string, robot_task_msgs::msg::RobotGoal> nodeList;
  std::vector<std::pair<std::string, std::string>> line_lists;

  // 私有函数声明（确保实现）
  void CmdSet();
  void RobotPoseCallBack(const nav_msgs::msg::Odometry::SharedPtr pose);
  void StateCallBack(const robot_state_msgs::msg::RobotState::SharedPtr data);
  void ClickPointCallBack(const geometry_msgs::msg::PointStamped::SharedPtr point);
  void disconnect_two_points(std::string first, std::string second);
  nlohmann::json convert_pose_to_json(geometry_msgs::msg::Pose pose);
  geometry_msgs::msg::Pose json_to_pose(const nlohmann::json &json_obj);
  visualization_msgs::msg::Marker convert_node_to_mark(int id, std::string name, geometry_msgs::msg::Pose &pose, int type);
  void connect_two_points(visualization_msgs::msg::Marker &line_list, geometry_msgs::msg::Pose &p1, geometry_msgs::msg::Pose &p2);
  void pub_node_list(void);
  void import_nodes(std::string fileName);
  void import_nodes(void);
  void import_edges(void);
};

} // namespace robot_control

#endif // ROBOT_CONTROL_H