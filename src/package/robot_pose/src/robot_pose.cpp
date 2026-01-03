
#include <std_msgs/String.h>
#include <sstream>
#include "../include/robot_pose/robot_pose.hpp"
#include <opencv2/opencv.hpp>

namespace robot_pose {

RobotPose::RobotPose(ros::NodeHandle nh, ros::NodeHandle pnh)
{
  /* read params */
  nhandle = nh;
  phandle = pnh;
  pubRobotPose = nhandle.advertise<nav_msgs::Odometry>("robot_pose", 1);
  pubRobotPoseInit = nhandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1,true);
  isRunning = true;
  poseSendThread = std::thread(&RobotPose::RobotPoseSend, this);
  sleep(3);
  SendRobotPose();
  initTimer = nhandle.createTimer(ros::Duration(2),&RobotPose::SaveRobotPose, this);
}

RobotPose::~RobotPose()
{
  boost::unique_lock<boost::shared_mutex> lockDeal(mutexInit);
  isRunning = false;
  if(poseSendThread.joinable())
    poseSendThread.join();
}

void RobotPose::SaveRobotPose(const ros::TimerEvent &event)
{
  boost::unique_lock<boost::shared_mutex> lockDeal(mutexInit);
  std::string dir = workspace_path+"/install/share/robot_pose/config/robot_pose.yaml";
  try
  {
    cv::FileStorage fs(dir,cv::FileStorage::WRITE);
    fs <<"x"<<robotPose.pose.pose.position.x;
    fs <<"y"<<robotPose.pose.pose.position.y;
    fs <<"z"<<robotPose.pose.pose.position.z;
    fs <<"rx"<<robotPose.pose.pose.orientation.x;
    fs <<"ry"<<robotPose.pose.pose.orientation.y;
    fs <<"rz"<<robotPose.pose.pose.orientation.z;
    fs <<"rw"<<robotPose.pose.pose.orientation.w;
    fs.release();
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  
  
}

void RobotPose::SendRobotPose(void)
{
  boost::unique_lock<boost::shared_mutex> lockDeal(mutexInit);
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header.frame_id = "map";
  pose.header.seq = 0;
  pose.header.stamp = ros::Time::now();
  std::string dir = workspace_path+"/install/share/robot_pose/config/robot_pose.yaml";
  try
  {
    cv::FileStorage fs(dir.c_str(),cv::FileStorage::READ);
    if(!fs.isOpened()){
      std::cout<<"Cant't open init file!"<<std::endl;
      return;
    }
    fs["x"]>>pose.pose.pose.position.x;
    fs["y"]>>pose.pose.pose.position.y;
    fs["z"]>>pose.pose.pose.position.z;
    fs["rx"]>>pose.pose.pose.orientation.x;
    fs["ry"]>>pose.pose.pose.orientation.y;
    fs["rz"]>>pose.pose.pose.orientation.z;
    fs["rw"]>>pose.pose.pose.orientation.w;
    pubRobotPoseInit.publish(pose);
    fs.release();
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  
}

void RobotPose::RobotPoseSend(void)
{
  ros::Rate rate(20);
  float transform_tolerance_ = 1.0;
  tf::TransformListener tf_listen;

  while (ros::ok() && isRunning)
  {

    ros::Time current_time = ros::Time::now();

    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.setIdentity();
    tf::Stamped <tf::Pose> base_pose;
    base_pose.setIdentity();
    base_pose.frame_id_ = "base_link";
    base_pose.stamp_ = ros::Time();


    // get the global pose of the robot
    try{
      tf_listen.transformPose("map", base_pose, robot_pose);
    }catch (tf::LookupException& ex){
      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
      rate.sleep();
      continue;
    }catch (tf::ConnectivityException& ex){
      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
      rate.sleep();
      continue;
    }catch (tf::ExtrapolationException& ex){
      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      rate.sleep();
      continue;
    }

    if (current_time.toSec() - robot_pose.stamp_.toSec() > transform_tolerance_)
    {
      ROS_WARN_THROTTLE(1.0,"Localization transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
               current_time.toSec(), robot_pose.stamp_.toSec(), transform_tolerance_);
      rate.sleep();
      continue;
    }

    {
     boost::unique_lock<boost::shared_mutex> lockDeal(mutexInit);
     robotPose = TransformToOdom(robot_pose,"map","base_link");;
    }
    pubRobotPose.publish(robotPose);
    rate.sleep();
  }
}

bool RobotPose::TransformFrame(nav_msgs::Odometry &s_pose,std::string goal_frame)
{
  tf::Stamped<tf::Pose> pose, global_pose;
  tf::poseMsgToTF(s_pose.pose.pose, pose);
  pose.frame_id_ = s_pose.header.frame_id;
  global_pose.frame_id_ = goal_frame;

  try{
    tfListener.transformPose(global_pose.frame_id_, pose, global_pose);
    s_pose.header.frame_id = goal_frame;
    s_pose.child_frame_id = s_pose.child_frame_id;
    s_pose.pose.pose.position.x = global_pose.getOrigin().getX();
    s_pose.pose.pose.position.y = global_pose.getOrigin().getY();
    s_pose.pose.pose.position.z = global_pose.getOrigin().getZ();
    s_pose.pose.pose.orientation.x= global_pose.getRotation().getX();
    s_pose.pose.pose.orientation.y= global_pose.getRotation().getY();
    s_pose.pose.pose.orientation.z= global_pose.getRotation().getZ();
    s_pose.pose.pose.orientation.w= global_pose.getRotation().getW();
    return true;
  }catch(tf::TransformException& ex){
    ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
             pose.frame_id_.c_str(), global_pose.frame_id_.c_str(), ex.what());
    return false;
  }
}

nav_msgs::Odometry RobotPose::TransformToOdom(tf::Stamped<tf::Pose> &pose,
                                              std::string frame,
                                              std::string child_frame)
{
  nav_msgs::Odometry ppose;
  ppose.header.frame_id = frame;
  ppose.child_frame_id = child_frame;
  ppose.header.stamp = ros::Time::now();

  ppose.pose.pose.position.x = pose.getOrigin().getX();
  ppose.pose.pose.position.y = pose.getOrigin().getY();
  ppose.pose.pose.position.z = 0;
  ppose.pose.pose.orientation.x = pose.getRotation().getX();
  ppose.pose.pose.orientation.y = pose.getRotation().getY();
  ppose.pose.pose.orientation.z = pose.getRotation().getZ();
  ppose.pose.pose.orientation.w = pose.getRotation().getW();

  ppose.pose.covariance[0]=1e-3;
  ppose.pose.covariance[7]=1e-3;
  ppose.pose.covariance[14]=1e-3;
  ppose.pose.covariance[21]=1e9;
  ppose.pose.covariance[28]=1e9;
  ppose.pose.covariance[35]=1e4;

  ppose.twist.covariance[0]=1e9;
  ppose.twist.covariance[7]=1e9;
  ppose.twist.covariance[14]=1e9;
  ppose.twist.covariance[21]=1e9;
  ppose.twist.covariance[28]=1e9;
  ppose.twist.covariance[35]=1e9;
  return ppose;
}
}

