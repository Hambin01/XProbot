
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef TCP_SERVICE_HPP_
#define TCP_SERVICE_HPP_

#include <iostream>
#include <string>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <boost/thread/pthread/shared_mutex.hpp>

#include <netinet/in.h> // sockaddr_in
#include <sys/types.h>  // socket
#include <sys/socket.h> // socket
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/select.h> // select
#include <sys/ioctl.h>
#include <sys/time.h>
#include <vector>
#include <map>

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

namespace tcp_com
{

  using namespace std;
#define BUFFER_SIZE 128
  bool isRunning = true;

  enum MsgType
  {
    HEART = 0,
    GO_GOAL,
    CROSS_RIGHT,
    CROSS_LEF
  };

  void *send_heart(void *arg);
  void *heart_handler(void *arg);

  class TcpService
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

    ros::Publisher pubTaskCmd;
    ros::Subscriber subTaskReq;
    ros::NodeHandle nhandle;
    ros::NodeHandle phandle;

  public:
    TcpService(ros::NodeHandle nh, ros::NodeHandle pnh, int port);
    ~TcpService();
    void Bind();
    void Listen(int queue_len = 20);
    void Accept();
    void Run();
    void Recv();
    void SendMsg(int fd, MsgType type, uint8_t data, int16_t x, int16_t y, int16_t height);
    void RobotTaskCallBack(robot_task_msgs::robot_task_reqConstPtr req);
    void DealMsg(int fd, uint8_t *msg, int num);
    friend void *heart_handler(void *arg);
    friend void *send_heart(void *arg);
  };

  TcpService::TcpService(ros::NodeHandle nh, ros::NodeHandle pnh, int port)
  {
    bzero(&server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htons(INADDR_ANY);
    server_addr.sin_port = htons(port);
    listen_fd = socket(PF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0)
    {
      ROS_ERROR("Create Socket Failed!");
      exit(1);
    }
    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    Bind();
    Listen();
    nhandle = nh;
    phandle = pnh;

    subTaskReq = nhandle.subscribe("/robot_task_req", 1, &TcpService::RobotTaskCallBack, this);
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
    if (max_fd < STDIN_FILENO)
    {
      max_fd = STDIN_FILENO;
    }
    msgThread = std::thread(&TcpService::Run, this);
  }

  TcpService::~TcpService()
  {
    isRunning = false;

    for (int fd = 0; fd <= max_fd; ++fd)
    {
      if (FD_ISSET(fd, &master_set))
      {
        close(fd);
      }
    }

    if (msgThread.joinable())
      msgThread.join();
    close(listen_fd);
  }

  void TcpService::Bind()
  {
    if (-1 == (bind(listen_fd, (struct sockaddr *)&server_addr, sizeof(server_addr))))
    {
      ROS_ERROR("Server Bind Failed!");
      exit(1);
    }
    cout << "绑定成功.\n";
  }

  void TcpService::Listen(int queue_len)
  {
    if (-1 == listen(listen_fd, queue_len))
    {
      ROS_ERROR("Server Listen Failed!");
      exit(1);
    }
    cout << "监听成功.\n";
  }

  void TcpService::Accept()
  {
    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);

    int new_fd = accept(listen_fd, (struct sockaddr *)&client_addr, &client_addr_len);
    if (new_fd < 0)
    {
      ROS_ERROR("Server Accept Failed!");
      exit(1);
    }

    string ip(inet_ntoa(client_addr.sin_addr));
    int port = htons(ntohs(client_addr.sin_port));
    ROS_INFO("新客户端接入: %s:%d", ip.c_str(), port);
    mmap.insert(make_pair(new_fd, make_pair(port, 0)));
    FD_SET(new_fd, &master_set);
    if (new_fd > max_fd)
      max_fd = new_fd;
  }

  void TcpService::Recv()
  {
    for (int fd = 0; fd <= max_fd; ++fd)
    {
      if (FD_ISSET(fd, &working_set))
      {
        bool close_conn = false;

        uint8_t data[128] = {0};
        int num = recv(fd, &data, sizeof(data), 0);
        if (num < 0)
        {
          ROS_ERROR("接收消息出错");
          close_conn = true;
        }
        else if (num == 0)
        {
          ROS_INFO("客户端 %d 退出", mmap[fd].first);
          close(fd);
          FD_CLR(fd, &master_set);
        }
        else
          DealMsg(fd, data, num);

        if (close_conn)
        {
          close(fd);
          FD_CLR(fd, &master_set);
          if (fd == max_fd)
          {
            while (FD_ISSET(max_fd, &master_set) == false)
              --max_fd;
          }
        }
      }
    }
  }

  void TcpService::Run()
  {
    while (isRunning)
    {
      FD_ZERO(&working_set);
      memcpy(&working_set, &master_set, sizeof(master_set));
      timeout.tv_sec = 2;
      int nums = select(max_fd + 1, &working_set, NULL, NULL, &timeout);
      if (nums < 0)
      {
        cout << "select() error!";
        // exit(1);
      }
      else if (nums == 0)
      {
        continue;
      }

      if (FD_ISSET(listen_fd, &working_set))
        Accept();
      else
        Recv();
    }
  }

  void *heart_handler(void *arg)
  {
    TcpService *s = (TcpService *)arg;
    int timeout = 10;

    while (isRunning)
    {
      map<int, pair<int, int>>::iterator it = s->mmap.begin();
      for (; it != s->mmap.end();)
      {
        if (it->second.second == timeout)
        {
          ROS_INFO("客户端 %d 掉线了", it->second.first);
          int fd = it->first;
          close(fd);
          FD_CLR(fd, &s->master_set);
          if (fd == s->max_fd)
          {
            while (FD_ISSET(s->max_fd, &s->master_set) == false)
              s->max_fd--;
          }
          s->mmap.erase(it++);
        }
        else if (it->second.second < timeout)
        {
          it->second.second++;
          ++it;
        }
        else
          ++it;
      }
      sleep(1);
    }
  }

  void *send_heart(void *arg)
  {
    TcpService *c = (TcpService *)arg;

    while (isRunning)
    {
      map<int, pair<int, int>>::iterator it = c->mmap.begin();
      for (; it != c->mmap.end();)
      {
        if (it->first >= 0)
        {
          c->SendMsg(it->first, HEART, 0x00, 0, 0, 0);
        }
        it++;
      }
      sleep(1);
    }
  }

  void TcpService::RobotTaskCallBack(robot_task_msgs::robot_task_reqConstPtr req)
  {
    SendMsg(req->fd, (MsgType)req->type, req->data, req->pose_x, req->pose_y, req->putter_height);
  }

  void TcpService::SendMsg(int fd, MsgType type, uint8_t data, int16_t x, int16_t y, int16_t height)
  {
    uint8_t msg[] = {0x0FC,
                     0x0B,
                     0x00,
                     0x00,
                     0x00, 0x00,
                     0x00, 0x00,
                     0x00, 0x00,
                     0x00,
                     0xCF};
    msg[2] = type;
    msg[3] = data;
    msg[4] = (x & 0xFF00) >> 8;
    msg[5] = x & 0x00FF;
    msg[6] = (y & 0xFF00) >> 8;
    msg[7] = y & 0x00FF;
    msg[8] = (height & 0xFF00) >> 8;
    msg[9] = height & 0x00FF;
    send(fd, &msg, sizeof(msg), 0);
  }

  void TcpService::DealMsg(int fd, uint8_t *msg, int num)
  {
    std::ostringstream oss;

    for (int i = 0; i < num; i++)
    {
      oss << std::hex << std::setw(2) << std::setfill('0') << (static_cast<unsigned int>(msg[i] & 0xff)) << " ";
    }
    std::string hex_str = oss.str();
    ROS_INFO("%s", hex_str.c_str());

    if (num != 12)
      return;

    if (msg[2] == 0x00)
    {
      mmap[fd].second = 0;
      return;
    }

    robot_task_msgs::robot_task_req send_cmd;
    send_cmd.fd = fd;
    send_cmd.type = msg[2];
    send_cmd.data = msg[3];
    send_cmd.putter_height = ((msg[8] | send_cmd.putter_height) << 8) | msg[9];

    pubTaskCmd.publish(send_cmd);
    ROS_INFO("robot received cmd type %d", send_cmd.type);
  }

}

#endif
