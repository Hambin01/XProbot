
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/tcp_com/tcp_com.hpp"


namespace tcp_com {

TcpCom::TcpCom(ros::NodeHandle nh, ros::NodeHandle pnh)
{
  //TcpService server(9000);//创建一个socket文件描述符

 // TcpClient tmpTcp("192.168.2.81",9000);
}

}

