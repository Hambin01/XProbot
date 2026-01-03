#include "motion_control/navigation_line.hpp"
#include "costmap_2d/costmap_math.h"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/twist.hpp"  // 新增cmd_vel消息头文件

namespace navigation
{

Navigation::Navigation(const rclcpp::NodeOptions &options)
: Node("navigation_node", options),
  action_type(STOP),
  maxSpeedFlag(false),
  navigation_pause(false),
  dir_type(0),
  run_adjust(0),
  use_run_adjust(true)
{
  // 初始化TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listen_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 声明并获取参数
  declare_parameters();
  get_parameters();

  // 创建订阅者
  adjust_req_subscriber_ = this->create_subscription<std_msgs::msg::Int8>(
    "/adjust_req",
    1,
    std::bind(&Navigation::AdjustReqCallback, this, std::placeholders::_1));

  navigation_cancel_subscriber_ = this->create_subscription<actionlib_msgs::msg::GoalID>(
    "/go_to_node/cancel",
    1,
    std::bind(&Navigation::NavigationCancelCallBack, this, std::placeholders::_1));

  robot_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/robot_pose",
    1,
    std::bind(&Navigation::RobotPoseCallback, this, std::placeholders::_1));

  robot_state_subscriber_ = this->create_subscription<robot_state_msgs::msg::RobotState>(
    "/robot_state",
    1,
    std::bind(&Navigation::StateCallBack, this, std::placeholders::_1));

  // 创建发布者
  navigation_speed_publisher_ = this->create_publisher<robot_state_msgs::msg::DataToStm32>(
    "/data_to_stm32",
    1);
  
  // 新增/cmd_vel发布者（ROS标准速度接口）
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    1);

  adjust_cmd_publisher_ = this->create_publisher<std_msgs::msg::Int8>(
    "/adjust_cmd",
    1);

  // 动态参数回调
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&Navigation::CfgCallback, this, std::placeholders::_1));

  // 创建Action服务器
  nav_server_ = rclcpp_action::create_server<RobotNavAction>(
    this,
    "go_to_node",
    std::bind(&Navigation::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&Navigation::handle_cancel, this, std::placeholders::_1),
    std::bind(&Navigation::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Navigation node initialized successfully");
}

Navigation::~Navigation()
{
  action_type = STOP;
  ResetSpeed();
  // 析构时发布零速度
  PublishCmdVel(0.0, 0.0, 0.0);
}

void Navigation::declare_parameters()
{
  // 声明所有参数，设置默认值
  this->declare_parameter("stop_distance", 0.1);
  this->declare_parameter("rotate_speed", 0.5);
  this->declare_parameter("max_speed", 0.3);
  this->declare_parameter("min_speed", 0.05);
  this->declare_parameter("stop_speed", 0.0);
  this->declare_parameter("stop_radio", 2.0);
  this->declare_parameter("max_pid", 1.0);
  this->declare_parameter("dis_radio", 0.5);
  this->declare_parameter("rotate_stop", 1.0);
  this->declare_parameter("dev_angle", 10.0);
  this->declare_parameter("stop_angle", 5.0);
  this->declare_parameter("localization_rate", 50.0);
  this->declare_parameter("goal_distance_threshold", 0.05);
  this->declare_parameter("goal_angle_threshold", 1.0);
  this->declare_parameter("adjust_distance", 0.06);
  this->declare_parameter("acc", 0.01);
  this->declare_parameter("kt", 0.8);
  
  // PID参数
  this->declare_parameter("kp", 1.0);
  this->declare_parameter("ki", 0.1);
  this->declare_parameter("kd", 0.05);
}

void Navigation::get_parameters()
{
  // 获取基础参数
  this->get_parameter("stop_distance", stop_distance);
  this->get_parameter("rotate_speed", rotate_speed);
  this->get_parameter("max_speed", max_speed);
  this->get_parameter("min_speed", min_speed);
  this->get_parameter("stop_speed", stop_speed);
  this->get_parameter("stop_radio", stop_radio);
  this->get_parameter("max_pid", max_pid);
  this->get_parameter("dis_radio", dis_radio);
  this->get_parameter("rotate_stop", rotate_stop);
  this->get_parameter("dev_angle", dev_angle);
  this->get_parameter("stop_angle", stop_angle);
  this->get_parameter("localization_rate", localization_rate);
  this->get_parameter("goal_distance_threshold", goal_distance_threshold);
  this->get_parameter("goal_angle_threshold", goal_angle_threshold);
  this->get_parameter("adjust_distance", adjust_distance);
  this->get_parameter("acc", acc);
  this->get_parameter("kt", kt);

  // 获取PID参数并初始化
  float kp, ki, kd;
  this->get_parameter("kp", kp);
  this->get_parameter("ki", ki);
  this->get_parameter("kd", kd);
  
  // 初始化PID（设置采样时间）
  pidCal.PidInit(kp, ki, kd, -max_pid, max_pid, 1.0/localization_rate);
  
  // 初始化卡尔曼滤波器
  k_angle = kalman::OneKalmanClass(0.01, 5.0);
  k_dis = kalman::OneKalmanClass(0.01, 5.0);
  k_t_dis = kalman::OneKalmanClass(0.01, 5.0);
}

// 新增：发布/cmd_vel消息的封装函数
void Navigation::PublishCmdVel(float linear_x, float linear_y, float angular_z)
{
  geometry_msgs::msg::Twist twist_msg;
  twist_msg.linear.x = linear_x;
  twist_msg.linear.y = linear_y;
  twist_msg.linear.z = 0.0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = angular_z;
  
  cmd_vel_publisher_->publish(twist_msg);
}

rcl_interfaces::msg::SetParametersResult Navigation::CfgCallback(
  const std::vector<rclcpp::Parameter> &params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "Success";

  for (const auto &param : params) {
    if (param.get_name() == "kp" || param.get_name() == "ki" || param.get_name() == "kd") {
      float kp = this->get_parameter("kp").as_double();
      float ki = this->get_parameter("ki").as_double();
      float kd = this->get_parameter("kd").as_double();
      pidCal.PidInit(kp, ki, kd, -max_pid, max_pid, 1.0/localization_rate);
    } else if (param.get_name() == "acc") {
      acc = param.as_double();
    } else if (param.get_name() == "kt") {
      kt = param.as_double();
    } else if (param.get_name() == "stop_distance") {
      stop_distance = param.as_double();
    } else if (param.get_name() == "rotate_speed") {
      rotate_speed = param.as_double();
    } else if (param.get_name() == "max_speed") {
      max_speed = param.as_double();
    } else if (param.get_name() == "min_speed") {
      min_speed = param.as_double();
    } else if (param.get_name() == "stop_speed") {
      stop_speed = param.as_double();
    } else if (param.get_name() == "stop_radio") {
      stop_radio = param.as_double();
    } else if (param.get_name() == "max_pid") {
      max_pid = param.as_double();
      // 重新初始化PID限幅
      float kp = this->get_parameter("kp").as_double();
      float ki = this->get_parameter("ki").as_double();
      float kd = this->get_parameter("kd").as_double();
      pidCal.PidInit(kp, ki, kd, -max_pid, max_pid, 1.0/localization_rate);
    } else if (param.get_name() == "dis_radio") {
      dis_radio = param.as_double();
    } else if (param.get_name() == "rotate_stop") {
      rotate_stop = param.as_double();
    } else if (param.get_name() == "dev_angle") {
      dev_angle = param.as_double();
    } else if (param.get_name() == "stop_angle") {
      stop_angle = param.as_double();
    } else if (param.get_name() == "localization_rate") {
      localization_rate = param.as_double();
      // 更新PID采样时间
      float kp = this->get_parameter("kp").as_double();
      float ki = this->get_parameter("ki").as_double();
      float kd = this->get_parameter("kd").as_double();
      pidCal.PidInit(kp, ki, kd, -max_pid, max_pid, 1.0/localization_rate);
    } else if (param.get_name() == "goal_distance_threshold") {
      goal_distance_threshold = param.as_double();
    } else if (param.get_name() == "goal_angle_threshold") {
      goal_angle_threshold = param.as_double();
    } else if (param.get_name() == "adjust_distance") {
      adjust_distance = param.as_double();
    }
  }

  return result;
}

void Navigation::RobotPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::shared_mutex> lockPose(mutexPose);
  robot_pose_ = *msg;
}

void Navigation::AdjustReqCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
  run_adjust = msg->data;
}

void Navigation::StateCallBack(const robot_state_msgs::msg::RobotState::SharedPtr msg)
{
  robotState = *msg;
}

void Navigation::NavigationCancelCallBack(const actionlib_msgs::msg::GoalID::SharedPtr msg)
{
  action_type = STOP;
  ResetParam();
  ResetSpeed();
  // 取消时发布零速度到/cmd_vel
  PublishCmdVel(0.0, 0.0, 0.0);
  
  if (current_goal_handle_) {
    auto result = std::make_shared<RobotNavAction::Result>();
    result->success = false;
    result->message = "Navigation canceled by user";
    current_goal_handle_->canceled(result);
    current_goal_handle_.reset();
  }
  
  RCLCPP_INFO(this->get_logger(), "Navigation canceled");
}

rclcpp_action::GoalResponse Navigation::handle_goal(
  const rclcpp_action::GoalUUID &uuid,
  std::shared_ptr<const RobotNavAction::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "Received navigation goal request");
  
  // 检查机器人状态
  if (!rclcpp::ok() || navigation_pause) {
    RCLCPP_WARN(this->get_logger(), "Navigation paused or node not ok, reject goal");
    return rclcpp_action::GoalResponse::REJECT;
  }
  
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Navigation::handle_cancel(
  const std::shared_ptr<GoalHandleRobotNav> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received navigation cancel request");
  action_type = STOP;
  ResetParam();
  ResetSpeed();
  // 取消时发布零速度到/cmd_vel
  PublishCmdVel(0.0, 0.0, 0.0);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Navigation::handle_accepted(const std::shared_ptr<GoalHandleRobotNav> goal_handle)
{
  // 在新线程中执行，避免阻塞回调
  std::thread{std::bind(&Navigation::Execute, this, std::placeholders::_1), goal_handle}.detach();
}

void Navigation::Execute(const std::shared_ptr<GoalHandleRobotNav> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<RobotNavAction::Result>();
  rclcpp::Rate rate(localization_rate);

  // 初始化状态
  action_type = ROTATE;
  goalPose = goal->goal_pose;
  startPose = goal->start_pose;
  
  // 解析目标姿态（提取yaw角）
  tf2::Quaternion quat;
  tf2::fromMsg(goalPose.pose.orientation, quat);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  goalPose.pose.position.z = yaw;

  // 获取当前机器人位姿
  geometry_msgs::msg::PoseStamped robot_pose;
  {
    std::lock_guard<std::shared_mutex> lockPose(mutexPose);
    robot_pose = robot_pose_;
  }

  // 初始距离检查
  double dist = hypot(
    goalPose.pose.position.x - robot_pose.pose.position.x,
    goalPose.pose.position.y - robot_pose.pose.position.y);
  
  double angle_diff = fabs(goalPose.pose.position.z - robot_pose.pose.position.z) * 180 / M_PI;
  double dist_thresh = (dir_type == 0) ? goal_distance_threshold : goal_distance_threshold * 2;
  double angle_thresh = (dir_type == 0) ? goal_angle_threshold : goal_angle_threshold * 2;

  if (dist < dist_thresh) {
    if (goal->goal_type != 1 && angle_diff > angle_thresh) {
      action_type = ROTATE_GOAL;
      RCLCPP_INFO(this->get_logger(), 
        "Initial check: ROTATE, distance=%.3f, angle_diff=%.2f", 
        dist, angle_diff);
    } else {
      // 已到达目标
      result->success = true;
      result->message = "Already at goal position";
      goal_handle->succeed(result);
      action_type = STOP;
      // 发布零速度
      PublishCmdVel(0.0, 0.0, 0.0);
      current_goal_handle_.reset();
      return;
    }
  }

  dir_type = goal->dir_type;
  int adjust_time = 0;
  RCLCPP_INFO(this->get_logger(), "Start navigation to goal, dir_type=%d", dir_type);

  // 主控制循环
  while (rclcpp::ok() && action_type != STOP) {
    // 检查是否被取消
    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Navigation canceled";
      goal_handle->canceled(result);
      action_type = STOP;
      // 发布零速度
      PublishCmdVel(0.0, 0.0, 0.0);
      current_goal_handle_.reset();
      return;
    }

    double abs_angle, dis, goal_angle, t_dis, t_angle, dx, dy;
    
    // 获取最新机器人位姿
    {
      std::lock_guard<std::shared_mutex> lockPose(mutexPose);
      robot_pose = robot_pose_;
    }

    // 计算各类偏差
    CalcAdjust(abs_angle, dis, goal_angle, t_dis, t_angle);

    dx = goalPose.pose.position.x - robot_pose.pose.position.x;
    dy = goalPose.pose.position.y - robot_pose.pose.position.y;

    // 计算前进方向符号
    int sig = sign0(
      distance(goalPose.pose.position.x, goalPose.pose.position.y, 
               startPose.pose.position.x, startPose.pose.position.y) -
      distance(robot_pose.pose.position.x, robot_pose.pose.position.y, 
               startPose.pose.position.x, startPose.pose.position.y));

    // 状态机处理
    switch (action_type)
    {
      case ROTATE: {
        // 出发时角度调整
        double adjust_angle = (dir_type == 0) ? abs_angle : 
          (abs_angle - 180 < -300 ? abs_angle + 180 : abs_angle - 180);
        
        if (RotateAdjust(adjust_angle, dir_type)) {
          action_type = GO_LINE;
          RCLCPP_INFO(this->get_logger(), "Rotation completed, start moving line");
        }
        break;
      }

      case GO_LINE: {
        // 直线行走控制
        if (fabs(t_dis) > adjust_distance) {
          // 横向偏差过大，重新调整角度
          action_type = ROTATE;
          startPose.pose.position.x = robot_pose.pose.position.x;
          startPose.pose.position.y = robot_pose.pose.position.y;
          startPose.pose.position.z = robot_pose.pose.position.z;
          ResetParam();
          RCLCPP_WARN(this->get_logger(), 
            "Lateral distance too big: %.3f, reset rotation", t_dis);
          break;
        }

        // 直线行走调整
        if (GoLineAdjust(abs_angle, t_dis, t_angle, sig * dis)) {
          if (goal->goal_type == 1) {
            // 到达目标
            action_type = STOP;
            ResetParam();
            // 发布零速度
            PublishCmdVel(0.0, 0.0, 0.0);
            result->success = true;
            result->message = "Reached goal (type 1)";
            goal_handle->succeed(result);
            current_goal_handle_.reset();
          } else {
            // 需要调整目标角度
            action_type = ROTATE_GOAL;
            RCLCPP_INFO(this->get_logger(), "Line movement completed, start goal rotation");
          }
        }
        break;
      }

      case ROTATE_GOAL: {
        // 目标点角度调整
        if (RotateAdjust(goal_angle, 0)) {
          action_type = RUN_ADJUST;
          ResetParam();
          RCLCPP_INFO(this->get_logger(), 
            "Goal rotation completed, dx=%.3f, dy=%.3f", dx, dy);
          
          run_adjust = 0;
          std_msgs::msg::Int8 adjust_cmd;
          adjust_cmd.data = 1;
          adjust_cmd_publisher_->publish(adjust_cmd);
        }
        break;
      }

      case RUN_ADJUST: {
        // 执行调整
        if (run_adjust == 1 || adjust_time > localization_rate * 20) {
          action_type = STOP;
          ResetParam();
          // 发布零速度
          PublishCmdVel(0.0, 0.0, 0.0);
          result->success = true;
          result->message = "Adjustment completed";
          goal_handle->succeed(result);
          current_goal_handle_.reset();
        }
        
        if (adjust_time > localization_rate * 20) {
          RCLCPP_WARN(this->get_logger(), "Run adjust timeout!");
        }

        adjust_time++;
        break;
      }

      case RUN_ERROR: {
        // 运行错误
        ResetParam();
        action_type = STOP;
        // 发布零速度
        PublishCmdVel(0.0, 0.0, 0.0);
        result->success = false;
        result->message = "Navigation error";
        goal_handle->abort(result);
        current_goal_handle_.reset();
        break;
      }

      default:
        break;
    }

    // 发布速度指令到stm32
    cmd.header.stamp = this->get_clock()->now();
    navigation_speed_publisher_->publish(cmd);
    
    // 同步发布速度指令到/cmd_vel（核心新增逻辑）
    PublishCmdVel(
      cmd.running.speed,    // 线速度x
      0.0,                  // 线速度y（根据需求可调整）
      cmd.running.radius    // 角速度z（原radius对应角速度）
    );
    
    // 发布反馈
    if (goal_handle->is_active()) {
      auto feedback = std::make_shared<RobotNavAction::Feedback>(navFeedback);
      goal_handle->publish_feedback(feedback);
    }

    rate.sleep();
  }

  // 完成导航
  if (action_type == STOP && goal_handle->is_active()) {
    // 发布零速度
    PublishCmdVel(0.0, 0.0, 0.0);
    result->success = true;
    result->message = "Navigation completed successfully";
    goal_handle->succeed(result);
    current_goal_handle_.reset();
  }
  
  RCLCPP_INFO(this->get_logger(), "Navigation execution finished");
}

void Navigation::ResetParam(void)
{
  // k_angle.Reset();
  // k_dis.Reset();
  // k_t_dis.Reset();
  pidCal.PidReset();
  maxSpeedFlag = false;
}

void Navigation::ResetSpeed(void)
{
  navFeedback.speed = 0;
  navFeedback.radius = 0;
  navFeedback.angle = 0;
  cmd.running.angle = 0;
  cmd.running.radius = 0;
  cmd.running.speed = 0;
  // 重置时同步发布零速度到/cmd_vel
  PublishCmdVel(0.0, 0.0, 0.0);
}

void Navigation::SetSpeed(float speed, float radius, float angle)
{
  cmd.task_type = 2;
  cmd.running.type = 1;
  cmd.running.angle = angle;
  cmd.running.radius = radius;
  cmd.running.speed = speed;
  // 设置速度时同步更新/cmd_vel（可选：也可在主循环统一发布）
  // PublishCmdVel(speed, 0.0, radius);
}

void Navigation::CalcAdjust(double &abs_angle,
                            double &dis,
                            double &goal_angle,
                            double &t_dis,
                            double &t_angle)
{
  geometry_msgs::msg::PoseStamped robot_pose;
  {
    std::lock_guard<std::shared_mutex> lockPose(mutexPose);
    robot_pose = robot_pose_;
  }

  // 计算角度偏差
  double robot_yaw = robot_pose.pose.position.z * 180 / M_PI;
  goal_angle = goalPose.pose.position.z * 180 / M_PI - robot_yaw;
  // 角度归一化到[-180, 180]
  goal_angle = (fabs(goal_angle) > 180) ? 
    (goal_angle > 0 ? goal_angle - 360 : goal_angle + 360) : goal_angle;

  // 计算机器人朝向
  tf2::Quaternion odom_quat = tf2::createQuaternionFromYaw((-robot_yaw + 90) * M_PI / 180);
  robot_yaw = tf2::getYaw(odom_quat) * 180 / M_PI;

  // 计算目标方向角度
  double angle = atan2(goalPose.pose.position.x - robot_pose.pose.position.x,
                       goalPose.pose.position.y - robot_pose.pose.position.y) * 180 / M_PI;
  
  abs_angle = robot_yaw - angle;
  // 角度归一化
  abs_angle = (fabs(abs_angle) > 180) ? 
    (abs_angle > 0 ? abs_angle - 360 : abs_angle + 360) : abs_angle;

  // 计算距离
  dis = distance(goalPose.pose.position.x, goalPose.pose.position.y,
                 robot_pose.pose.position.x, robot_pose.pose.position.y);

  // 计算横向偏差
  t_dis = GetDistanceToLine(startPose, goalPose, robot_pose) *
          distanceToLine(robot_pose.pose.position.x, robot_pose.pose.position.y,
                         startPose.pose.position.x, startPose.pose.position.y,
                         goalPose.pose.position.x, goalPose.pose.position.y);

  // 计算相对角度
  t_angle = GetAngelOfTwoVector(goalPose, robot_pose, startPose);
  t_angle = sign0(t_angle) * ((fabs(t_angle) > 90) ? 180 - fabs(t_angle) : fabs(t_angle));

  // 可选：卡尔曼滤波
  // abs_angle = k_angle.Update(abs_angle);
  // dis = k_dis.Update(dis);
  // t_dis = k_t_dis.Update(t_dis);
}

bool Navigation::RotateAdjust(double angle, int dir)
{
  navFeedback.robot_action_type = 2; // ROTATE
  
  if (fabs(angle) > rotate_stop) {
    // 计算旋转速度
    navFeedback.speed = 0;
    navFeedback.radius = fabs(angle) > dev_angle ? 
      (dir == 0 ? rotate_speed : rotate_speed / 2) : min_speed;
    navFeedback.radius = fabs(angle) > stop_angle ? navFeedback.radius : stop_speed;
    navFeedback.radius = -sign0(angle) * navFeedback.radius;
    
    // 设置速度指令
    SetSpeed(navFeedback.speed, navFeedback.radius, 0);
    return false;
  } else {
    // 旋转完成
    ResetSpeed();
    return true;
  }
}

bool Navigation::TransAdjust(double distance)
{
  navFeedback.robot_action_type = 3; // TRANS
  
  if (fabs(distance) > 0.012) {
    navFeedback.speed = -sign0(distance) * 0.05;
    SetSpeed(navFeedback.speed, navFeedback.radius, 0);
    return false;
  } else {
    ResetSpeed();
    return true;
  }
}

bool Navigation::GoLineAdjust(double angle, double t_dis, double t_angle, double distance)
{
  static double curSpeed = 0;
  static float maxSpeed;
  float dev_distance;

  navFeedback.robot_action_type = 4; // GO_LINE
  navFeedback.radius = 0;

  // 初始化最大速度
  if (!maxSpeedFlag) {
    maxSpeed = sqrt(fabs(acc * distance));
    maxSpeed = std::clamp(maxSpeed, min_speed, max_speed);
    maxSpeedFlag = true;
  }

  if (distance > stop_distance) {
    // 计算减速距离
    dev_distance = maxSpeed * maxSpeed / (stop_radio * acc);
    
    // 速度规划
    if (dir_type == 0) {
      curSpeed = NeedAcc(distance > dev_distance ? maxSpeed : min_speed, navFeedback.speed);
    } else {
      curSpeed = -NeedAcc(distance > dev_distance ? maxSpeed / 2.0 : min_speed, navFeedback.speed);
    }

    // 计算控制误差
    float cal_err;
    if (distance > dev_distance / 4) {
      cal_err = -kt * angle / 10 + (1 - kt) * t_dis / 0.1;
    } else {
      cal_err = kt * dis_radio * t_dis / 0.1;
    }

    // PID控制
    navFeedback.speed = curSpeed;
    navFeedback.radius = navFeedback.speed * pidCal.PidCal(dir_type == 0 ? cal_err : 0);
    
    // 设置速度指令
    SetSpeed(navFeedback.speed, navFeedback.radius, 0);
    return false;
  } else {
    // 到达停止距离
    ResetSpeed();
    curSpeed = 0;
    maxSpeedFlag = false;
    return true;
  }
}

double Navigation::NeedAcc(double setSpeed, double curSpeed)
{
  setSpeed = fabs(setSpeed);
  curSpeed = fabs(curSpeed);
  double speedCnt = setSpeed - curSpeed;

  // 加速度控制
  double delta_speed = sign0(speedCnt) * acc / (localization_rate * 1.0f);
  curSpeed += delta_speed;

  // 速度限幅
  if (speedCnt > 0) {
    curSpeed = std::min(curSpeed, setSpeed);
  } else if (speedCnt < 0) {
    curSpeed = std::max(curSpeed, setSpeed);
  }

  return curSpeed;
}

float Navigation::GetAngelOfTwoVector(geometry_msgs::msg::PoseStamped &pt1,
                                      geometry_msgs::msg::PoseStamped &pt2,
                                      geometry_msgs::msg::PoseStamped &c)
{
  // 计算两个向量的夹角
  float theta = atan2(pt1.pose.position.x - c.pose.position.x, 
                      pt1.pose.position.y - c.pose.position.y) -
                atan2(pt2.pose.position.x - c.pose.position.x, 
                      pt2.pose.position.y - c.pose.position.y);

  // 转换为角度并归一化
  theta = theta * 180.0 / M_PI;
  theta = (fabs(theta) > 180) ? 
    (theta > 0 ? theta - 360 : theta + 360) : theta;

  return theta;
}

double Navigation::GetDistanceToLine(geometry_msgs::msg::PoseStamped &pt1,
                                     geometry_msgs::msg::PoseStamped &pt2,
                                     geometry_msgs::msg::PoseStamped &c)
{
  // 计算点到直线的位置（左侧/右侧）
  double A = pt1.pose.position.x - c.pose.position.x;
  double B = pt2.pose.position.y - c.pose.position.y;
  double C = pt2.pose.position.x - c.pose.position.x;
  double D = pt1.pose.position.y - c.pose.position.y;

  double dot = A * B - C * D;

  return sign0(dot);
}

} // namespace navigation

// ROS 2节点注册
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(navigation::Navigation)