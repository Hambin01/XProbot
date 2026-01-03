
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef JOY_F710_HPP_
#define JOY_F710_HPP_

#include <iostream>
#include <string>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <boost/thread/pthread/shared_mutex.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>

namespace transform_util {

bool TransformFrame(geometry_msgs::PoseStamped &s_pose,
                    geometry_msgs::PoseStamped &goal_pose,
                    std::string goal_frame,
                    tf::TransformListener &tfListener)
{
  tf::Stamped<tf::Pose> pose, global_pose;
  tf::poseMsgToTF(s_pose.pose, pose);
  pose.frame_id_ = s_pose.header.frame_id;
  global_pose.frame_id_ = goal_frame;

  try{
    tfListener.transformPose(global_pose.frame_id_, pose, global_pose);
    goal_pose.header.frame_id = goal_frame;
    goal_pose.pose.position.x = global_pose.getOrigin().getX();
    goal_pose.pose.position.y = global_pose.getOrigin().getY();
    goal_pose.pose.position.z = global_pose.getOrigin().getZ();
    goal_pose.pose.orientation.x= global_pose.getRotation().getX();
    goal_pose.pose.orientation.y= global_pose.getRotation().getY();
    goal_pose.pose.orientation.z= global_pose.getRotation().getZ();
    goal_pose.pose.orientation.w= global_pose.getRotation().getW();
    return true;
  }catch(tf::TransformException& ex){
    ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
             pose.frame_id_.c_str(), global_pose.frame_id_.c_str(), ex.what());
    return false;
  }
}

}
#endif
