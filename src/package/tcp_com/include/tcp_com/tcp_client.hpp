
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef TCP_CLIENT_HPP_
#define TCP_CLIENT_HPP_

#include <iostream>
#include <string>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <boost/thread/pthread/shared_mutex.hpp>
#include <ros/ros.h>

#include<errno.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<netinet/in.h>
#include<unistd.h>
#include<arpa/inet.h>
#include<fcntl.h>

namespace tcp_com {

class TcpClient
{
public:
  TcpClient(std::string name, uint16_t port);
  virtual ~TcpClient(void);
  int ConnectServer(void);
  void Disconnect();

  int sendMessage();
  int recvMessage();
private:
  struct sockaddr_in m_ConnAddr;
  int clientSock = -1;
  bool isRev;
  bool isConnected = false;
  std::string name;
  uint16_t port;
};


int TcpClient :: ConnectServer(void)
{
  if(name.empty()){
    return -1;
  }
  clientSock = socket(AF_INET,SOCK_STREAM,0);
  if(clientSock < 0){
    ROS_ERROR("sock\n");
    return -1;
  }
  m_ConnAddr.sin_family = AF_INET;
  m_ConnAddr.sin_port = htons(port);
  m_ConnAddr.sin_addr.s_addr = inet_addr(name.c_str());

  int iLen = sizeof(m_ConnAddr);
  int iRet = connect(clientSock,(struct sockaddr *)&m_ConnAddr,iLen);
  if(iRet < 0){
    ROS_ERROR("connect error!\n");
    return -1;
  }else{
    ROS_INFO("ConnectServer IP = %s, port = %d sucess\n",name.c_str(),port);
    isConnected = true;
    return 1;
  }
}


TcpClient::TcpClient(std::string n,uint16_t t)
{
  name = n;
  port=t;
  ConnectServer();
}

TcpClient::~TcpClient(void)
{
  Disconnect();
}

void TcpClient::Disconnect()
{
  close(clientSock);
  clientSock = -1;
}

int TcpClient::sendMessage()
{
  int iRet ;
  char szSendbuf[256];
  //sROS_INFO(szSendbuf,"01 03 09 00 00 06 C6 54");
  if(isConnected)
  {
    iRet = send(clientSock,szSendbuf,sizeof(szSendbuf),0);
    if(iRet > 0)
    {
      ROS_INFO("send message %s success\n",szSendbuf);
    }
    else
    {
      ROS_INFO("send message %s failed\n",szSendbuf);
    }
  }
  return iRet;
}

int TcpClient::recvMessage()
{
  int iRet;
  unsigned char szRecvbuf[256];
  iRet = recv(clientSock,szRecvbuf,sizeof(szRecvbuf),0);
  szRecvbuf[iRet] = 0;
  if(isConnected)
  {
    if(iRet < 0)
    {
      ROS_INFO("recv data failed\n");
    }
    else
    {
      ROS_INFO("recv %d data  success\n",iRet);
      int iDataNum = 0;
      while(szRecvbuf[iDataNum])
      {
        ROS_INFO("%02X ",szRecvbuf[iDataNum++]);
      }
      ROS_INFO("\n");
    }
  }
  return iRet;
}

//end  class TcpClient

}
#endif
