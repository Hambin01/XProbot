
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef MODBUS_SERVICE_HPP_
#define MODBUS_SERVICE_HPP_

#include <iostream>
#include <string>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <boost/thread/pthread/shared_mutex.hpp>

#include<netinet/in.h>   // sockaddr_in
#include<sys/types.h>    // socket
#include<sys/socket.h>   // socket
#include<arpa/inet.h>
#include<unistd.h>
#include<sys/select.h>   // select
#include<sys/ioctl.h>
#include<sys/time.h>
#include<vector>
#include<map>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <robot_state_msgs/robot_state.h>
#include <robot_state_msgs/data_to_stm32.h>
#include <robot_state_msgs/charge_action.h>
#include <robot_state_msgs/putter_action.h>
#include <robot_state_msgs/running_action.h>
#include <robot_task_msgs/robot_goal.h>
#include <robot_task_msgs/robot_task_req.h>

namespace modbus_com {

using namespace std;
#define BUFFER_SIZE 128
bool isRunning = true;

enum MsgType{
  RIGHT_UP =14,
  LEFT_DOWN,
  LEFT_UP,
  RIGHT_DOWN
};


enum CodeType{
  CROSS_STATE =2,
  GET_STATE =4,
  SET_CROSS,
  SET_NODE
};

void* send_heart(void* arg);
void* heart_handler(void* arg);

class ModbusService
{

private:
  struct sockaddr_in server_addr;
  socklen_t server_addr_len;
  int listen_fd;
  int max_fd;
  fd_set master_set;
  fd_set working_set;
  struct timeval timeout;
  map<int, pair<int, int>> mmap;
  boost::shared_mutex mutexMsg;
  std::thread msgThread;
  uint8_t msgfd[5];

  ros::Publisher  pubTaskCmd;
  ros::Subscriber subTaskReq;
  ros::NodeHandle nhandle;
  ros::NodeHandle phandle;

  int cross_type;
  map<int,list<uint8_t>> client_id;

public:
  ModbusService(ros::NodeHandle nh, ros::NodeHandle pnh,int port);
  ~ModbusService();
  void Bind();
  void Listen(int queue_len = 20);
  void Accept();
  void Run();
  void Recv();
  void SendMsg(int fd, int type, uint8_t data, int16_t x, int16_t y, int16_t height);
  void RobotTaskCallBack(robot_task_msgs::robot_task_reqConstPtr req);
  void DealMsg(int fd, uint8_t *msg, int num);
  friend void* heart_handler(void* arg);
  friend void* send_heart(void* arg);
};

ModbusService::ModbusService(ros::NodeHandle nh, ros::NodeHandle pnh,int port)
{
  bzero(&server_addr, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = htons(INADDR_ANY);
  server_addr.sin_port = htons(port);
  listen_fd = socket(PF_INET, SOCK_STREAM, 0);
  if(listen_fd < 0){
    ROS_ERROR("Create Socket Failed!");
    exit(1);
  }
  int opt = 1;
  setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  Bind();
  Listen();
  nhandle = nh;
  phandle = pnh;

  subTaskReq = nhandle.subscribe("/robot_task_req", 1, &ModbusService::RobotTaskCallBack, this);
  pubTaskCmd = nhandle.advertise<robot_task_msgs::robot_task_req>("/robot_task_cmd", 1);

  /*pthread_t id;
  int ret = pthread_create(&id, NULL, heart_handler, (void*)this);
  if(ret != 0){
    cout << "Can not create heart-beat checking thread.\n";
    exit(1);
  }

  pthread_t id1;
  int ret1 = pthread_create(&id, NULL, send_heart, (void*)this);
  if(ret1 != 0){
    cout << "Can not create send_heart thread!";
    exit(1);
  }*/

  max_fd = listen_fd;
  FD_ZERO(&master_set);
  FD_SET(listen_fd, &master_set);
  FD_SET(STDIN_FILENO, &master_set);
  if(max_fd <STDIN_FILENO){
    max_fd = STDIN_FILENO;
  }
  msgThread = std::thread(&ModbusService::Run, this);
}

ModbusService::~ModbusService()
{
  isRunning = false;

  for(int fd=0; fd<=max_fd; ++fd){
    if(FD_ISSET(fd, &master_set)){
      close(fd);
    }
  }

  if(msgThread.joinable())
    msgThread.join();
  close(listen_fd);
}

void ModbusService::Bind()
{
  if(-1 == (bind(listen_fd, (struct sockaddr*)&server_addr, sizeof(server_addr))))
  {
    ROS_ERROR("Server Bind Failed!");
    exit(1);
  }
  cout << "绑定成功.\n";
}

void ModbusService::Listen(int queue_len)
{
  if(-1 == listen(listen_fd, queue_len)){
    ROS_ERROR("Server Listen Failed!");
    exit(1);
  }
  cout << "监听成功.\n";
}

void ModbusService::Accept()
{
  struct sockaddr_in client_addr;
  socklen_t client_addr_len = sizeof(client_addr);

  int new_fd = accept(listen_fd, (struct sockaddr*)&client_addr, &client_addr_len);
  if(new_fd < 0)
  {
    ROS_ERROR("Server Accept Failed!");
    exit(1);
  }

  string ip(inet_ntoa(client_addr.sin_addr));
  int port = htons(ntohs(client_addr.sin_port));
  cout << ip << "connection was accepted!\n";
  mmap.insert(make_pair(new_fd, make_pair(port, 0)));
  FD_SET(new_fd, &master_set);
  if(new_fd > max_fd)
    max_fd = new_fd;
}

void ModbusService::Recv()
{
  for(int fd=0; fd<=max_fd; ++fd){
    if(FD_ISSET(fd, &working_set)){
      bool close_conn = false;

      uint8_t data[128]={0};
      int num=recv(fd, &data, sizeof(data), 0);
      if(num<0){
        ROS_ERROR("接收消息出错");
        close_conn = true;
      }
      else if(num==0){
        cout<<"客户端"<<mmap[fd].first<<"退出"<<endl;
        close(fd);
        FD_CLR(fd, &master_set);
      }
      else
        DealMsg(fd,data,num);

      if(close_conn){
        close(fd);
        FD_CLR(fd, &master_set);
        if(fd == max_fd){
          while(FD_ISSET(max_fd, &master_set) == false)
            --max_fd;
        }
      }
    }
  }
}

void ModbusService::Run()
{
  while(isRunning)
  {
    FD_ZERO(&working_set);
    memcpy(&working_set, &master_set, sizeof(master_set));
    timeout.tv_sec = 2;
    int nums = select(max_fd+1, &working_set, NULL, NULL, &timeout);
    if(nums < 0){
      cout << "select() error!";
      //exit(1);
    }else if(nums == 0){
      continue;
    }

    if(FD_ISSET(listen_fd, &working_set))
      Accept();
    else
      Recv();

  }

}

void* heart_handler(void* arg)
{
  ModbusService* s = (ModbusService*)arg;
  int timeout = 10;

  while(isRunning)
  {
    map<int, pair<int, int> >::iterator it = s->mmap.begin();
    for( ; it!=s->mmap.end(); )
    {
      if(it->second.second == timeout){
        cout << "客户端 " << it->second.first << " 掉线了\n";
        int fd = it->first;
        close(fd);
        FD_CLR(fd, &s->master_set);
        if(fd == s->max_fd) {
          while(FD_ISSET(s->max_fd, &s->master_set) == false)
            s->max_fd--;
        }
        s->mmap.erase(it++);
      }else if(it->second.second < timeout){
        it->second.second++;
        ++it;
      }else
        ++it;
    }
    sleep(1);
  }
}

void* send_heart(void* arg)
{
  ModbusService* c = (ModbusService*)arg;

  while(isRunning)
  {
    map<int, pair<int, int> >::iterator it = c->mmap.begin();
    for( ; it!=c->mmap.end(); ){
      if(it->first>=0){
        c->SendMsg(it->first,0,0x00,0,0,0);
      }
      it++;
    }
    sleep(1);
  }
}


void ModbusService::RobotTaskCallBack(robot_task_msgs::robot_task_reqConstPtr req)
{
  SendMsg(req->fd,req->type,req->data,req->pose_x,req->pose_y,req->putter_height);
}

void ModbusService::SendMsg(int fd,int type,uint8_t data,int16_t x,int16_t y,int16_t height)
{
  uint8_t msg[12];
  uint8_t task_state_msg[17]={00,00,00,00,00,0x0B,01,04,0x08,00,0x0a,00,01,00,0x0b,00,03};
  uint8_t cross_state_msg[10]={00,00,00,00,00,04,01,02,01,00};
  uint8_t task_state_msg2[11]={00,00,00,00,00,05,01,04,02,00,04};
  //协议头
  msg[0] = 0;//前两位为事务处理标识，2个字节的序列号
  msg[1] = 0;
  msg[2] = 0;//协议标识，两个字节，0000即为Modbus TCP协议
  msg[3] = 0;
  msg[4] = 0;//长度字段（前半部分字节）
  msg[5] = 6;//随后字节数
  msg[6] = 0x01;//单元标志，定义连接目的节点的其他设备
  //协议体
  int index=0;
  for(auto iter=client_id[fd].begin();iter!=client_id[fd].end();iter++){
    msg[index]=*iter;
    index++;
  }
  switch (type) {
  case 0x0b:
    msg[7] = 0x05;
    msg[8] = 0x00;
    msg[9] = cross_type;

    if(data == 2){
      msg[10] = 0x00;
      msg[10] = 0x00;
    }
    memcpy(msg,msgfd,5);
    send(fd, &msg, sizeof(msg), 0);
    break;

    case 0x0c:
    task_state_msg[10]=data ;
    task_state_msg[12]=y;
    task_state_msg[14]=x;
    memcpy(task_state_msg,msgfd,5);
    send(fd, &task_state_msg, sizeof(task_state_msg), 0);
    break;

    case 0x0d:
    cross_state_msg[9]=height&0x01;
    memcpy(cross_state_msg,msgfd,5);
    send(fd, &cross_state_msg, sizeof(cross_state_msg), 0);
    break;

    case 0x0E:
    task_state_msg2[10]=data;
    memcpy(task_state_msg2,msgfd,5);
    send(fd, &task_state_msg2, sizeof(task_state_msg2), 0);
    break;
    
    //pwq
    case 0X0F:
    cross_state_msg[9]=height&0x01;
    memcpy(cross_state_msg,msgfd,5);
    send(fd, &cross_state_msg, sizeof(cross_state_msg), 0);
    break;
  default:
    break;
  }

  //send(fd, &msg, sizeof(msg), 0);
}


void ModbusService::DealMsg(int fd,uint8_t *msg,int num)
{
  for(int i=0;i<num;i++)
    std::cout << std::hex << (msg[i] & 0xff) << " ";
  std::cout<<std::endl;
  uint8_t rec_msg[12];
  memcpy(rec_msg,msg,12);
  memcpy(msgfd,msg,5);
  bool should_public=true;
  
  if(num!=12)
    return;

  // if(msg[2]==0x00){
  //   mmap[fd].second=0;
  //   return;
  // }

  robot_task_msgs::robot_task_req send_cmd;
  send_cmd.fd = fd;

  list<uint8_t> msgfd_list;
  for(int i=0;i<5;i++){
    msgfd_list.push_back(msgfd[i]);
  }
  if(client_id.find(fd)==client_id.end()){
    client_id.insert(pair<int,list<uint8_t>>(fd,msgfd_list));
  }else{
    client_id[fd]=msgfd_list;
  }

  switch (msg[7]) {
  case CROSS_STATE:
    //pwq
    //send_cmd.type = 0x0D;
    send_cmd.type = 0x0F;
    break;

  case GET_STATE:
    if(msg[9] == 0x06) {//30007
      send_cmd.type = 0x0C;
    }else  if(msg[9] == 0x08) {//30009}
      send_cmd.type = 0x0E;
    }
    break;

  case SET_CROSS:
    send(fd, &rec_msg, sizeof(rec_msg), 0);
    send_cmd.type = 0x02;
    cross_type = msg[9];
    if(msg[10] != 0x00)
    {
      switch (msg[9]) {
      case LEFT_UP:
        send_cmd.data = 0x02;
        break;
      case LEFT_DOWN:
        send_cmd.data = 0x04;
        break;
      case RIGHT_UP:
        send_cmd.data = 0x01;
        break;
      case RIGHT_DOWN:
        send_cmd.data = 0x03;
        break;
      default:
        break;
      }
    }else should_public=false;
    break;

  case SET_NODE:
    ROS_INFO("robot received SET_NODE");
    send(fd, &rec_msg, sizeof(rec_msg), 0);
    send_cmd.type = 0x01;
    send_cmd.data = ((msg[10]|send_cmd.data)<<8)|msg[11];
    break;
  default:
    break;
  }

  if(should_public)
  pubTaskCmd.publish(send_cmd);
  ROS_INFO("robot received cmd type %d",send_cmd.type);
}

}

#endif
