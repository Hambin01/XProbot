#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
ros::Subscriber initial_pose_sub_;
ros::Publisher  initial_pose_publisher;
ros::ServiceClient client_traj_finish;
ros::ServiceClient client_traj_start;
//int counter_init_pose;
int traj_id;
bool use_cartographer = false;

void init_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{

  traj_id++;

  cartographer_ros_msgs::FinishTrajectory srv_traj_finish;
  srv_traj_finish.request.trajectory_id = traj_id;
  ROS_INFO("finish_trajectory: %d", traj_id);
  if(client_traj_finish.call(srv_traj_finish))
  {
    ROS_INFO("call finish_trajectory %d success!", traj_id);
  }
  else
  {
    ROS_INFO("failed to call finish_trajectory service!");
  }


  cartographer_ros_msgs::StartTrajectory srv_traj_start;
  srv_traj_start.request.configuration_directory = "/home/robot/ROBOT_INDOOR/install/share/cartographer_ros/configuration_files";
  srv_traj_start.request.configuration_basename = "localization.lua";
  srv_traj_start.request.use_initial_pose = true;
  srv_traj_start.request.initial_pose = msg->pose.pose;
  srv_traj_start.request.relative_to_trajectory_id = 0;
  if(client_traj_start.call(srv_traj_start))
  {
    ROS_INFO("call start_trajactory %d success!", traj_id);
  }
  else
  {
    ROS_INFO("failed to call start_trajectory service!");
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "initial_pose_node");
    ros::NodeHandle n;

    n.getParam("/init_pose/use_cartographer", use_cartographer);
    client_traj_finish = n.serviceClient<cartographer_ros_msgs::FinishTrajectory>("finish_trajectory");
    client_traj_start = n.serviceClient<cartographer_ros_msgs::StartTrajectory>("start_trajectory");
    initial_pose_sub_ = n.subscribe("/initialpose", 1, init_pose_callback);
    initial_pose_publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1);
    ros::spin();
    return 0;
}
