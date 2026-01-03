/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef NAVIGATION_LINE_HPP_
#define NAVIGATION_LINE_HPP_

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <memory>
#include <shared_mutex>
#include <cmath>

// ROS 2核心头文件
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int8.hpp"
#include "actionlib_msgs/msg/goal_id.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/twist.hpp"

// 自定义组件
#include "kalman.hpp"
#include "pid.hpp"  // 需确保PID类适配ROS 2
#include "motion_control/action/robot_nav.hpp"  // ROS 2 action接口
#include "motion_control/param/motion_control_param.hpp"  // 动态参数配置
#include "robot_state_msgs/msg/robot_state.hpp"  // 自定义消息
#include "robot_state_msgs/msg/data_to_stm32.hpp"  // 自定义消息
#include "motion_control/glog_util.hpp"  // 日志工具

namespace navigation
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

  class Navigation : public rclcpp::Node
  {
  public:
    /**
     * @brief 构造函数
     * @param options ROS 2节点配置选项
     */
    explicit Navigation(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    
    /**
     * @brief 析构函数
     */
    virtual ~Navigation();

  private:
    // ROS 2订阅者
    rclcpp::Subscription<actionlib_msgs::msg::GoalID>::SharedPtr navigation_cancel_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr adjust_req_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_subscriber_;
    rclcpp::Subscription<robot_state_msgs::msg::RobotState>::SharedPtr robot_state_subscriber_;

    // ROS 2发布者
    rclcpp::Publisher<robot_state_msgs::msg::DataToStm32>::SharedPtr navigation_speed_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr adjust_cmd_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    // 动态参数回调句柄
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // 滤波和PID控制器
    kalman::OneKalmanClass k_angle;
    kalman::OneKalmanClass k_dis;
    kalman::OneKalmanClass k_t_dis;
    pid::PidClass pidCal;

    // 控制参数
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

    // 状态变量
    bool maxSpeedFlag = false;
    bool navigation_pause;
    int dir_type = 0;
    int run_adjust = 0;
    bool use_run_adjust = true;

    // 机器人状态
    robot_state_msgs::msg::RobotState robotState;
    Adjust_Type action_type = STOP;
    geometry_msgs::msg::PoseStamped goalPose;
    geometry_msgs::msg::PoseStamped startPose;

    // 线程安全锁
    std::shared_mutex mutexPose;
    geometry_msgs::msg::PoseStamped robot_pose_;

    // TF2相关
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listen_;

    // 控制指令
    robot_state_msgs::msg::DataToStm32 cmd;

    // ROS 2 Action服务器
    using RobotNavAction = motion_control::action::RobotNav;
    using GoalHandleRobotNav = rclcpp_action::ServerGoalHandle<RobotNavAction>;
    rclcpp_action::Server<RobotNavAction>::SharedPtr nav_server_;
    std::shared_ptr<GoalHandleRobotNav> current_goal_handle_;

    // 核心回调函数
    /**
     * @brief 动态参数配置回调
     * @param params 参数列表
     * @return 参数设置结果
     */
    rcl_interfaces::msg::SetParametersResult CfgCallback(
      const std::vector<rclcpp::Parameter> &params);

    /**
     * @brief 调整请求回调
     * @param msg 调整请求消息
     */
    void AdjustReqCallback(const std_msgs::msg::Int8::SharedPtr msg);

    /**
     * @brief 机器人状态回调
     * @param msg 机器人状态消息
     */
    void StateCallBack(const robot_state_msgs::msg::RobotState::SharedPtr msg);

    /**
     * @brief 导航取消回调
     * @param msg 取消目标消息
     */
    void NavigationCancelCallBack(const actionlib_msgs::msg::GoalID::SharedPtr msg);

    /**
     * @brief 机器人位姿回调
     * @param msg 位姿消息
     */
    void RobotPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void PublishCmdVel(float linear_x, float linear_y, float angular_z);

    // Action服务器处理函数
    /**
     * @brief 处理Action目标请求
     * @param uuid 目标UUID
     * @param goal 目标数据
     * @return 目标响应
     */
    rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const RobotNavAction::Goal> goal);

    /**
     * @brief 处理Action取消请求
     * @param goal_handle 目标句柄
     * @return 取消响应
     */
    rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleRobotNav> goal_handle);

    /**
     * @brief 处理已接受的Action目标
     * @param goal_handle 目标句柄
     */
    void handle_accepted(const std::shared_ptr<GoalHandleRobotNav> goal_handle);

    /**
     * @brief 核心执行逻辑
     * @param goal_handle 目标句柄
     */
    void Execute(const std::shared_ptr<GoalHandleRobotNav> goal_handle);

    // 控制算法函数
    /**
     * @brief 旋转调整
     * @param angle 偏差角度
     * @param dir 方向类型
     * @return 是否调整完成
     */
    bool RotateAdjust(double angle, int dir);

    /**
     * @brief 平移调整
     * @param distance 偏差距离
     * @return 是否调整完成
     */
    bool TransAdjust(double distance);

    /**
     * @brief 直线行走调整
     * @param angle 角度偏差
     * @param t_dis 横向距离偏差
     * @param t_angle 相对角度偏差
     * @param distance 剩余距离
     * @return 是否到达目标
     */
    bool GoLineAdjust(double angle, double t_dis, double t_angle, double distance);

    /**
     * @brief 计算两个向量的夹角
     * @param pt1 点1
     * @param pt2 点2
     * @param c 中心点
     * @return 夹角(度)
     */
    float GetAngelOfTwoVector(geometry_msgs::msg::PoseStamped &pt1,
                              geometry_msgs::msg::PoseStamped &pt2,
                              geometry_msgs::msg::PoseStamped &c);

    /**
     * @brief 计算点到直线的距离符号
     * @param pt1 直线点1
     * @param pt2 直线点2
     * @param c 目标点
     * @return 距离符号
     */
    double GetDistanceToLine(geometry_msgs::msg::PoseStamped &pt1,
                             geometry_msgs::msg::PoseStamped &pt2,
                             geometry_msgs::msg::PoseStamped &c);

    /**
     * @brief 加速度控制
     * @param setSpeed 目标速度
     * @param curSpeed 当前速度
     * @return 调整后的速度
     */
    double NeedAcc(double setSpeed, double curSpeed);

    /**
     * @brief 计算各类偏差
     * @param abs_angle 绝对角度偏差
     * @param dis 距离偏差
     * @param goal_angle 目标角度偏差
     * @param t_dis 横向距离偏差
     * @param t_angle 相对角度偏差
     */
    void CalcAdjust(double &abs_angle,
                    double &dis,
                    double &goal_angle,
                    double &t_dis,
                    double &t_angle);

    /**
     * @brief 重置控制参数
     */
    void ResetParam(void);

    /**
     * @brief 重置速度指令
     */
    void ResetSpeed(void);

    /**
     * @brief 设置速度指令
     * @param speed 线速度
     * @param radius 角速度/曲率
     * @param angle 角度
     */
    void SetSpeed(float speed, float radius, float angle);

    /**
     * @brief 声明ROS 2参数
     */
    void declare_parameters();

    /**
     * @brief 获取ROS 2参数
     */
    void get_parameters();

    /**
     * @brief 符号函数
     * @param x 输入值
     * @return 1(正)/0(零)/-1(负)
     */
    template <typename T>
    int sign0(T x)
    {
      return (x > 0) ? 1 : (x < 0) ? -1 : 0;
    }

    /**
     * @brief 计算两点间距离
     * @param x1 点1x
     * @param y1 点1y
     * @param x2 点2x
     * @param y2 点2y
     * @return 距离
     */
    double distance(double x1, double y1, double x2, double y2)
    {
      return hypot(x2 - x1, y2 - y1);
    }

    /**
     * @brief 计算点到直线的距离
     * @param x0 点x
     * @param y0 点y
     * @param x1 直线点1x
     * @param y1 直线点1y
     * @param x2 直线点2x
     * @param y2 直线点2y
     * @return 距离
     */
    double distanceToLine(double x0, double y0, double x1, double y1, double x2, double y2)
    {
      double A = y2 - y1;
      double B = x1 - x2;
      double C = x2 * y1 - x1 * y2;
      return fabs(A * x0 + B * y0 + C) / hypot(A, B);
    }
  };

}  // namespace navigation

#endif  // NAVIGATION_LINE_HPP_