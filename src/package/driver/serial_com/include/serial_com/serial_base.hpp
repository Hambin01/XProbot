
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef SERIAL_BASE_HPP_
#define SERIAL_BASE_HPP_

#include <iostream>
#include <string>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <boost/thread/pthread/shared_mutex.hpp>
#include <glog/logging.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <queue>

namespace serial_com {

class SerialBase {
public:  
  enum RobotDownloadCode
  {
    NO_IDLE =0,
    IDLE		,
    START_DOWNLOAD			,
    DOWNLOAD,
    UPDATE,
    WAITING,
    FINISH
  };

public:
  SerialBase();
  virtual ~SerialBase();

  ros::NodeHandle nhandle;
  boost::shared_mutex mutexRead;
  boost::shared_mutex mutexWrite;

  virtual void DataDeal(uint8_t *data, size_t length)=0;

protected:
  bool isRunning=false;
  serial::Serial _com;
  std::string _com_name;
  std::queue<std::vector<uint8_t>> data_cmd;
  std::vector<uint8_t>  rec_data;
  RobotDownloadCode download_state=NO_IDLE;
  std::string bin_file = "";

  void Run(void);
  bool OpenSerial(void);
  bool CloseSerial(void);
  void SendData(std::vector<uint8_t> &data);
  void ReceivedData(void);
  void Download(uint8_t *buffer, size_t len);

  template <typename T>
  T Uint8ToConvert(std::vector<uint8_t> &byte, int &count,T &invert)
  {
    if(typeid(invert).name() == typeid(int32_t).name() || typeid(invert).name() == typeid(uint32_t).name())
    {
      std::cout<<"";
      invert = ((((((invert&0x00000000)|byte[count++]<<8)|byte[count++])<<8)|byte[count++])<<8)|byte[count++];
      return invert;
    }
    else if(typeid(invert).name() == typeid(int16_t).name() || typeid(invert).name() == typeid(uint16_t).name()){
      std::cout<<"";
      invert = ((invert&0x0000)|byte[count++]<<8)|byte[count++];
      return invert;
    }
    else
      std::cout<<"no type!"<<std::endl;
  }

  template <typename T>
  void ConvertToUint8(T iValue,std::vector<uint8_t>& bytes)
  {
    if(typeid(iValue).name() == typeid(int32_t).name() || typeid(iValue).name() == typeid(uint32_t).name())
    {
      bytes.push_back((uint8_t)((iValue & 0xFF000000) >> 24));
      bytes.push_back((uint8_t) ((iValue & 0x00FF0000) >> 16));
      bytes.push_back((uint8_t) ((iValue & 0x0000FF00) >> 8));
      bytes.push_back((uint8_t) (iValue & 0x000000FF));
    } else if(typeid(iValue).name() == typeid(int16_t).name() || typeid(iValue).name() == typeid(uint16_t).name()){
      bytes.push_back((uint8_t) ((iValue & 0xFF00) >> 8));
      bytes.push_back((uint8_t) (iValue & 0x00FF));
    }else
      std::cout<<"no type!"<<std::endl;
  }

  uint16_t  Crc16Check(const std::vector<uint8_t> &data, int index, int len);


private:   
  std::thread receivedThread;


};

}
#endif

