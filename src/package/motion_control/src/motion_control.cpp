
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>

#include "../include/motion_control/motion_control.hpp"
#include <costmap_2d/costmap_math.h>

namespace navigation
{

  Navigation::Navigation(ros::NodeHandle nh, ros::NodeHandle pnh) : nav_server(nh, "go_to_node", boost::bind(&Navigation::Execute, this, _1), false)
  {
    /* read params */
    nhandle = nh;
    phandle = pnh;

    subRobotState = phandle.subscribe("/robot_state", 1, &Navigation::StateCallBack, this);
    adjust_req_subscriber_ = nh.subscribe("/adjust_req", 1, &Navigation::AdjustReqCallback, this);
    navigation_cancel_subscriber_ = phandle.subscribe("/go_to_node/cancel", 1, &Navigation::NavigationCancelCallBack, this);
    robot_pose_subscriber_ = nh.subscribe("/robot_pose", 1, &Navigation::RobotPoseCallback, this);

    navigation_speed_publisher_ = nh.advertise<robot_state_msgs::data_to_stm32>("/data_to_stm32", 1);
    adjust_cmd_publisher_ = nh.advertise<std_msgs::Int8>("/adjust_cmd", 1);

    f = boost::bind(&Navigation::CfgCallback, this, _1, _2);
    server.setCallback(f);
    nav_server.start();
    pnh.getParam("/stop_distance", stop_distance);
    pnh.getParam("/rotate_speed", rotate_speed);
    pnh.getParam("/max_speed", max_speed);
    pnh.getParam("/min_speed", min_speed);
    pnh.getParam("/stop_speed", stop_speed);
    pnh.getParam("/stop_radio", stop_radio);
    pnh.getParam("/max_pid", max_pid);
    pnh.getParam("/dis_radio", dis_radio);
    pnh.getParam("/rotate_stop", rotate_stop);
    pnh.getParam("/dev_angle", dev_angle);
    pnh.getParam("/stop_angle", stop_angle);
    pnh.getParam("/localization_rate", localization_rate);
    pnh.getParam("/goal_distance_threshold", goal_distance_threshold);
    pnh.getParam("/goal_angle_threshold", goal_angle_threshold);
    pnh.getParam("/adjust_distance", adjust_distance);
  }

  Navigation::~Navigation()
  {

  }

  // 动态参数
  void Navigation::CfgCallback(motion_control::motion_control_cfgConfig &cfg, uint32_t level)
  {

    // pid参数
    pidCal.PidInit(cfg.kp, cfg.ki, cfg.kd, -cfg.max_pid, cfg.max_pid);
    // 加速度
    acc = cfg.acc;
    // 计算间隔
    kt = cfg.kt;
    // 停止距离
    stop_distance = cfg.stop_distance;
    // 最大旋转速度
    rotate_speed = cfg.rotate_speed;
    // 最大速度
    max_speed = cfg.max_speed;

    min_speed = cfg.min_speed;

    stop_speed = cfg.stop_speed;
    // 停止比例
    stop_radio = cfg.stop_radio;
    // 最大pid
    max_pid = cfg.max_pid;
    // 距离比例
    dis_radio = cfg.dis_radio;

    rotate_stop = cfg.rotate_stop;

    dev_angle = cfg.dev_angle;

    stop_angle = cfg.stop_angle;

    localization_rate = cfg.localization_rate;

    goal_distance_threshold = cfg.goal_distance_threshold;

    goal_angle_threshold = cfg.goal_angle_threshold;

    adjust_distance = cfg.adjust_distance;
  }

  void Navigation::StateCallBack(const robot_state_msgs::robot_stateConstPtr &msg)
  {
    robotState = *msg;
  }

  void Navigation::RobotPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    robot_pose_ = *msg;
  }

  void Navigation::AdjustReqCallback(const std_msgs::Int8ConstPtr &msg)
  {
    run_adjust = msg->data;
  }

  void Navigation::NavigationCancelCallBack(const actionlib_msgs::GoalID &msg)
  {
    action_type = STOP;
    ResetParam();
    ResetSpeed();
    nav_server.setPreempted();
  }
  // 两点之间运动执行
  void Navigation::Execute(const motion_control::robotNavGoalConstPtr &goal)
  {
    ros::Rate rate(localization_rate);

    action_type = ROTATE;
    goalPose = goal->goal_pose;
    startPose = goal->start_pose;
    tf::Quaternion quat;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(goalPose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    double goal_yaw=yaw;

    geometry_msgs::PoseStamped robot_pose;
    {
      boost::unique_lock<boost::shared_mutex> lockPose(mutexPose);
      robot_pose = robot_pose_;
    }
    tf::quaternionMsgToTF(goalPose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    double robot_yaw = yaw;

    if (hypot(goalPose.pose.position.x - robot_pose.pose.position.x, goalPose.pose.position.y - robot_pose.pose.position.y) < (dir_type == 0 ? goal_distance_threshold : goal_distance_threshold * 2))
    {
      if (goal->goal_type != 1 && fabs(goal_yaw - robot_yaw) * 180 / M_PI > (dir_type == 0 ? goal_angle_threshold : goal_angle_threshold * 2))
      {
        action_type = ROTATE_GOAL;
        std::cout << "ROTATE:" << hypot(goalPose.pose.position.x - robot_pose.pose.position.x, goalPose.pose.position.y - robot_pose_.pose.position.y) << std::endl;
        std::cout << "GOAL:" << fabs(goal_yaw - robot_yaw) * 180 / M_PI << std::endl;
      }
      else
      {
        nav_server.setSucceeded();
        action_type = STOP;
      }
    }
    dir_type = goal->dir_type;
    int adjust_time = 0;
    // LOG(INFO)<<"start go to node!";

    while (action_type != STOP)
    {
      double abs_angle, dis, goal_angle, t_dis, t_angle, dx, dy;
      geometry_msgs::PoseStamped robot_pose;
      {
        boost::unique_lock<boost::shared_mutex> lockPose(mutexPose);
        robot_pose = robot_pose_;
      }

      // 计算偏差
      CalcAdjust(abs_angle, dis, goal_angle, t_dis, t_angle);

      dx = goalPose.pose.position.x - robot_pose.pose.position.x;
      dy = goalPose.pose.position.y - robot_pose.pose.position.y;

      int sig = sign0(distance(goalPose.pose.position.x, goalPose.pose.position.y, startPose.pose.position.x, startPose.pose.position.y) -
                      distance(robot_pose.pose.position.x, robot_pose.pose.position.y, startPose.pose.position.x, startPose.pose.position.y));

      switch (action_type)
      {

      // 出发时调整角度
      case ROTATE:
        if (dir_type == 0 ? RotateAdjust(abs_angle, 0) : RotateAdjust(abs_angle - 180 < -300 ? abs_angle + 180 : abs_angle - 180, 1))
          action_type = GO_LINE;
        break;

        // 直行
      case GO_LINE:
      {
        // 横向偏差大于0.06m则停下调整位姿
        if (fabs(t_dis) > adjust_distance)
        {
          action_type = ROTATE;
          startPose.pose.position.x = robot_pose.pose.position.x;
          startPose.pose.position.y = robot_pose.pose.position.y;
          ResetParam();
          // ROS_ERROR("dis is too big!dis:%f",t_dis);
          break;
        }
        if (GoLineAdjust(abs_angle, t_dis, t_angle, sig * dis))
        {
          if (goal->goal_type == 1)
          {
            action_type = STOP;
            ResetParam();
            nav_server.setSucceeded();
          }
          else
            action_type = ROTATE_GOAL;
        }
      }
      break;

        // 到目标点调整角度
      case ROTATE_GOAL:
        if (RotateAdjust(goal_angle, 0))
        {
          action_type = RUN_ADJUST;
          ResetParam();
          LOG(INFO) << ":dx:" << dx;
          LOG(INFO) << "dy:" << dy;
          LOG(INFO) << "***************************";
          // nav_server.setSucceeded();
          run_adjust = 0;
          std_msgs::Int8 adjust_cmd;
          adjust_cmd.data = 1;
          adjust_cmd_publisher_.publish(adjust_cmd);
        }
        break;

      case RUN_ADJUST:
        if (run_adjust == 1 || adjust_time > localization_rate * 20)
        {
          action_type = STOP;
          ResetParam();
          nav_server.setSucceeded();
        }
        if (adjust_time > localization_rate * 20)
          LOG(WARNING) << "Run adjust timeout!";

        adjust_time++;
        break;

        // 运行中断
      case RUN_ERROR:
        ResetParam();
        action_type = STOP;
        nav_server.setAborted();
        break;

      default:
        break;
      }
      cmd.header.stamp = ros::Time::now();
      navigation_speed_publisher_.publish(cmd);
      nav_server.publishFeedback(navFeedback);
      rate.sleep();
    }
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
  }

  void Navigation::SetSpeed(float speed, float radius, float angle)
  {
    cmd.task_type = 2;
    cmd.running.type = 1;
    cmd.running.angle = angle;
    cmd.running.radius = -radius;
    cmd.running.speed = speed;
  }

  // 计算偏差
  void Navigation::CalcAdjust(double &abs_angle,
                              double &dis,
                              double &goal_angle,
                              double &t_dis,
                              double &t_angle)
  {
    geometry_msgs::PoseStamped robot_pose;
    {
      boost::unique_lock<boost::shared_mutex> lockPose(mutexPose);
      robot_pose = robot_pose_;
    }
    tf::Quaternion quat;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(robot_pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    double robot_yaw = yaw * 180 / M_PI;
    tf::quaternionMsgToTF(robot_pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    goal_angle = yaw * 180 / M_PI - robot_yaw;
    goal_angle = (fabs(goal_angle) > 180) ? (goal_angle > 0 ? goal_angle - 360 : goal_angle + 360) : goal_angle; // 目标点角度
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw((-robot_yaw + 90) * M_PI / 180);
    robot_yaw = tf::getYaw(odom_quat) * 180 / M_PI; // 机器人当前角度
    //std::cout << "robot_yaw:" <<robot_yaw<< std::endl;

    double angle = atan2(goalPose.pose.position.x - robot_pose.pose.position.x, goalPose.pose.position.y - robot_pose.pose.position.y) * 180 / M_PI;
    abs_angle = robot_yaw - angle;
    abs_angle = (fabs(abs_angle) > 180) ? (abs_angle > 0 ? abs_angle - 360 : abs_angle + 360) : abs_angle; // 机器人与目标点连线角度

    dis = distance(goalPose.pose.position.x, goalPose.pose.position.y, robot_pose.pose.position.x, robot_pose.pose.position.y); // 与目标点之间距离

    t_dis = GetDistanceToLine(startPose, goalPose, robot_pose) *
            distanceToLine(robot_pose.pose.position.x, robot_pose.pose.position.y,
                           startPose.pose.position.x, startPose.pose.position.y,
                           goalPose.pose.position.x, goalPose.pose.position.y); // 横向偏差
    t_angle = GetAngelOfTwoVector(goalPose, robot_pose, startPose);
    t_angle = sign0(t_angle) * ((fabs(t_angle) > 90) ? 180 - fabs(t_angle) : fabs(t_angle)); // 相对角度

    // abs_angle=k_angle.Update(abs_angle); //滤波
    // dis=k_dis.Update(dis);//滤波
    // t_dis=k_t_dis.Update(t_dis);//滤波
  }

  bool Navigation::RotateAdjust(double angle, int dir)
  {
    navFeedback.robot_action_type = 2;
    if (fabs(angle) > rotate_stop)
    {
      navFeedback.speed = 0;
      navFeedback.radius = fabs(angle) > dev_angle ? (dir == 0 ? rotate_speed : rotate_speed / 2) : min_speed;
      navFeedback.radius = fabs(angle) > stop_angle ? navFeedback.radius : stop_speed;
      navFeedback.radius = -sign0(angle) * navFeedback.radius;
      SetSpeed(navFeedback.speed, navFeedback.radius, 0);
       //std::cout<<angle<<std::endl;
      return false;
    }
    else
    {
      ResetSpeed();
      return true;
    }
  }

  // 横移调整
  bool Navigation::TransAdjust(double distance)
  {
    navFeedback.robot_action_type = 3;
    if (distance > 0.012)
    {
      navFeedback.speed = -sign0(distance) * 0.05;
      SetSpeed(navFeedback.speed, navFeedback.radius, 0);
      return false;
    }
    else
    {
      ResetSpeed();
      return true;
    }
  }

  bool Navigation::GoLineAdjust(double angle, double t_dis, double t_angle, double distance)
  {
    static double curSpeed = 0;
    static float maxSpeed;
    float dev_distance;

    navFeedback.robot_action_type = 4;
    navFeedback.radius = 0;
    // acc = navFeedback.speed/(max_speed*2)+0.08;

    if (!maxSpeedFlag)
    {
      maxSpeed = sqrt(fabs(acc * distance));
      maxSpeed = maxSpeed > max_speed ? max_speed : (maxSpeed < min_speed ? min_speed : maxSpeed);
      maxSpeedFlag = true;
    }

    if (distance > stop_distance)
    {
      dev_distance = maxSpeed * maxSpeed / (stop_radio * acc);
      curSpeed = dir_type == 0 ? NeedAcc(distance > dev_distance ? maxSpeed : min_speed, navFeedback.speed) : -NeedAcc(distance > dev_distance ? maxSpeed / 2.0 : min_speed, navFeedback.speed);
      float cal_err;
      if (distance > dev_distance / 4)
        cal_err = -kt * angle / 10 + (1 - kt) * t_dis / 0.1;
      else
        cal_err = -0 * angle / 10 + kt * dis_radio * t_dis / 0.1;

      navFeedback.speed = curSpeed;
      // navFeedback.radius = dir_type==0?navFeedback.speed*pidCal.PidCal(cal_err):-navFeedback.speed*pidCal.PidCal(cal_err);
      navFeedback.radius = navFeedback.speed * pidCal.PidCal(dir_type == 0 ? cal_err : 0);
      SetSpeed(navFeedback.speed, navFeedback.radius, 0);
      return false;
    }
    else
    {
      ResetSpeed();
      curSpeed = 0;
      return true;
    }
  }

  // 自定义加速方式
  double Navigation::NeedAcc(double setSpeed, double curSpeed)
  {
    setSpeed = fabs(setSpeed);
    curSpeed = fabs(curSpeed);
    double speedCnt = setSpeed - curSpeed;

    curSpeed = curSpeed + sign0(speedCnt) * acc / (localization_rate * 1.0f);
    if (speedCnt > 0)
      curSpeed = curSpeed > setSpeed ? setSpeed : curSpeed;
    else if (speedCnt < 0)
      curSpeed = curSpeed < setSpeed ? setSpeed : curSpeed;

    return curSpeed;
  }

  float Navigation::GetAngelOfTwoVector(geometry_msgs::PoseStamped &pt1,
                                        geometry_msgs::PoseStamped &pt2,
                                        geometry_msgs::PoseStamped &c)
  {
    float theta = atan2(pt1.pose.position.x - c.pose.position.x, pt1.pose.position.y - c.pose.position.y) - atan2(pt2.pose.position.x - c.pose.position.x, pt2.pose.position.y - c.pose.position.y);

    theta = theta * 180.0 / M_PI;
    theta = (fabs(theta) > 180) ? (theta > 0 ? theta - 360 : theta + 360) : theta;

    return theta;
  }

  double Navigation::GetDistanceToLine(geometry_msgs::PoseStamped &pt1,
                                       geometry_msgs::PoseStamped &pt2,
                                       geometry_msgs::PoseStamped &c)
  {
    double A = pt1.pose.position.x - c.pose.position.x;
    double B = pt2.pose.position.y - c.pose.position.y;
    double C = pt2.pose.position.x - c.pose.position.x;
    double D = pt1.pose.position.y - c.pose.position.y;

    double dot = A * B - C * D;

    return sign0(dot);
  }
}
