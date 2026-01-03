/*
 * @Author: your name
 * @Date: 2021-07-19 15:03:57
 * @LastEditTime: 2021-08-02 19:17:27
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /xm-autosystem/src/package/application/robot_control/src/robot_control_node.cpp
 */

#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/String.h>

ros::Subscriber text_subscriber;
void TextCallback(const std_msgs::StringConstPtr& msg)
{
  FILE *fp;
  std::string text = "ekho "+msg->data;
  fp = popen(text.c_str(), "r");
  std::cout<<"tts_text:%s"<<msg->data<<std::endl;
  int res = pclose(fp);
  if(res == -1)
    ROS_ERROR("close error!");
  else
    fp =NULL;
}

void ShutDown(int signum) {
    ros::shutdown();
    exit(-1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tts_node");
    ros::NodeHandle nh;

    text_subscriber = nh.subscribe("/tts_text", 2, TextCallback);
    signal(SIGINT, ShutDown);
    ROS_INFO("tts_node start!");
    ros::spin();
    ros::shutdown();
    return 0;
    

}
