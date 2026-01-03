
#include "../include/odom/odom.h"

namespace odom
{

  Odom::Odom(ros::NodeHandle &nh)
  {
    nh_ = nh;
    last_time1 = ros::Time::now();
    robot_state_subscriber_ = nh_.subscribe("/robot_state", 1, &Odom::StateHandle, this);
    odom_publisher_ = nh_.advertise<nav_msgs::Odometry>("/odom", 1);

    nh_.getParam("/vth_scale", vth_scale);
    nh_.getParam("/vx_scale", vx_scale);
    nh_.getParam("/vy_scale", vy_scale);
    nh_.getParam("/sendTransform", sendTransform);
  }

  Odom::~Odom()
  {
  }

  void Odom::StateHandle(const robot_state_msgs::robot_stateConstPtr &msg)
  {
    robot_state_msgs::robot_state robotState = *msg;
    current_time1 = ros::Time::now();
    double vx, vy, vth;

    vx = vx_scale * robotState.speed;
    vy = 0;
    vth = -vth_scale * robotState.radius;
    double dt = (current_time1 - last_time1).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;
    x += delta_x;
    y += delta_y;
    th += delta_th;

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time1;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

    // send the transform
    if (sendTransform == 1)
      odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time1;
    odom.header.frame_id = "odom";

    // set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);

    // set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    odom.pose.covariance[0] = 0.1;   // x
    odom.pose.covariance[7] = 0.1;   // y
    odom.pose.covariance[35] = 0.1;  // yaw
    odom.twist.covariance[0] = 0.01; // vx
    odom.twist.covariance[7] = 0.01; // vy
    odom.twist.covariance[35] = 0.1; // vz

    // publish the message
    odom_publisher_.publish(odom);
    last_time1 = current_time1;
  }
} // namespace odom
