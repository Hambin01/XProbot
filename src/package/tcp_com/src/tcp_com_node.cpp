
#include "../include/tcp_com/tcp_service.hpp"
#include "../include/tcp_com/modbus_service.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "tcp_com_node");
  ros::NodeHandle pHandle("~");
  ros::NodeHandle gHandle;

  int tcp_port1 = 0;
  int tcp_port2 = 0;
  int modbus_port = 0; 
  int modbus_port1 = 0; 
  int modbus_port2 = 0; 
  pHandle.getParam("/tcp_com/tcp_port1", tcp_port1);
  pHandle.getParam("/tcp_com/tcp_port2", tcp_port2);
  pHandle.getParam("/tcp_com/modbus_port", modbus_port);
  pHandle.getParam("/tcp_com/modbus_port1", modbus_port1);
  pHandle.getParam("/tcp_com/modbus_port2", modbus_port2);

  if(tcp_port1!=0 && tcp_port2!=0){
    tcp_com::TcpService server(gHandle,pHandle,tcp_port1);
    tcp_com::TcpService server2(gHandle,pHandle,tcp_port2);
    ros::spin();
    return 0;
  }
  
  if(modbus_port!=0&&modbus_port1!=0&&modbus_port2!=0){
    modbus_com::ModbusService server3(gHandle,pHandle,modbus_port);
    //modbus_com::ModbusService server4(gHandle,pHandle,modbus_port1);
    //modbus_com::ModbusService server5(gHandle,pHandle,modbus_port2);
    std::cout<<"modbus port:"<<modbus_port<<std::endl;
    ros::spin();
    return 0;
  }
  
  
}
