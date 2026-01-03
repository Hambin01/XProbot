#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <robot_state_msgs/data_to_stm32.h>
#include <robot_state_msgs/charge_action.h>
#include <robot_state_msgs/putter_action.h>
#include <robot_state_msgs/running_action.h>

// 全局发布器（避免回调函数中频繁创建）
ros::Publisher twist_pub;

/**
 * @brief data_to_stm32话题回调函数
 * @param msg 接收到的data_to_stm32类型消息
 */
void dataToStm32Callback(const robot_state_msgs::data_to_stm32::ConstPtr &msg)
{
  geometry_msgs::Twist twist_msg;

  if (msg->task_type == 2 && msg->running.type == 1)
  {

    twist_msg.linear.x = msg->running.speed;  // 线速度（前进/后退）
    twist_msg.angular.z = msg->running.angle; // 角速度（转向角度映射为角速度）

    twist_pub.publish(twist_msg);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_to_twist_node");
  ros::NodeHandle nh;

  twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Subscriber sub = nh.subscribe("/data_to_stm32", 10, dataToStm32Callback);

  ROS_INFO("Data to Twist node started. Listening to /data_to_stm32, publishing to cmd_vel");

  ros::spin();

  return 0;
}