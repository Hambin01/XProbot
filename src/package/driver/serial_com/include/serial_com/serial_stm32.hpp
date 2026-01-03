
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef SERIAL_STM32_HPP_
#define SERIAL_STM32_HPP_

#include <iostream>
#include <string>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <boost/thread/pthread/shared_mutex.hpp>

#include <ros/ros.h>
#include <serial/serial.h>
#include <serial_com/serial_base.hpp>
#include <robot_state_msgs/robot_state.h>
#include <robot_state_msgs/data_to_stm32.h>
#include <robot_state_msgs/charge_action.h>
#include <robot_state_msgs/cross_action.h>
#include <robot_state_msgs/putter_action.h>
#include <robot_state_msgs/running_action.h>

namespace serial_com {

class SerialToStm32 :public SerialBase {
  enum RobotTaskCode
  {
    RobotStatus			=1,
    RobotMoving			,
    RobotTroubleReset,
    RobotInitialize,
    RobotTempPause,
    RobotOpenCharge,
    RobotCloseCharge,
    RobotSetParams,
    RobotQueryParams,
    RobotDownload,
    RobotTimeAdjust,
    RobotSleep,
    RobotCross,
    RobotPutter,
    RobotPutterInit
  };


public:
  SerialToStm32(ros::NodeHandle nh,std::string name);
  virtual ~SerialToStm32();



private:
  ros::Publisher  pubRobotState;
  ros::Publisher  pubStateGet;
  ros::Subscriber subSendData;
  boost::shared_mutex mutexDeal;
  std::thread dealThread;
  ros::Timer  stateTimer;
  ros::Timer  dataTimer;

  void DataDeal(uint8_t *data, size_t length);
  void CmdDeal(void);
  void SendDataCallBack(const robot_state_msgs::data_to_stm32ConstPtr);
  void StateGet(const ros::TimerEvent &event);
  void DataGet(const ros::TimerEvent& event);
  void InfoShow(std::vector<uint8_t> &data, size_t length);

};

}
#endif
