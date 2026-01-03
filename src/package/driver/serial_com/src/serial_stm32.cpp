
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/serial_com/serial_stm32.hpp"

namespace serial_com
{

  SerialToStm32::SerialToStm32(ros::NodeHandle nh, std::string name)
  {
    /* read params */
    nhandle = nh;
    _com_name = name;
    pubRobotState = nhandle.advertise<robot_state_msgs::robot_state>("robot_state", 1);
    pubStateGet = nhandle.advertise<robot_state_msgs::data_to_stm32>("data_to_stm32", 1);
    subSendData = nhandle.subscribe("/data_to_stm32", 1, &SerialToStm32::SendDataCallBack, this);
    Run();
    if (!isRunning)
      return;
    dealThread = std::thread(&SerialToStm32::CmdDeal, this);
   // stateTimer = nhandle.createTimer(ros::Duration(0.04), &SerialToStm32::StateGet, this);
    dataTimer = nhandle.createTimer(ros::Duration(0.01), &SerialToStm32::DataGet, this);
  }

  SerialToStm32::~SerialToStm32()
  {
    isRunning = false;
    if (dealThread.joinable())
      dealThread.join();
  }

  void SerialToStm32::SendDataCallBack(const robot_state_msgs::data_to_stm32ConstPtr data)
  {
    std::vector<uint8_t> cmd;

    cmd.push_back(0xfc);
    switch (data->task_type)
    {
    case RobotStatus:
    case RobotTroubleReset:
    case RobotInitialize:
    case RobotTempPause:
    case RobotCloseCharge:
    case RobotSleep:
    case RobotQueryParams:
    case RobotPutterInit:
      cmd.push_back(8);
      cmd.push_back(data->task_type);
      break;
      
    case RobotDownload:
      download_state = IDLE;
      cmd.push_back(8);
      cmd.push_back(data->task_type);
      break;

    case RobotMoving:
      cmd.push_back(21);
      cmd.push_back(data->task_type);
      cmd.push_back(data->running.type);
      ConvertToUint8((int32_t)(data->running.speed * 1000), cmd);
      ConvertToUint8((int32_t)(data->running.radius * 1000), cmd);
      ConvertToUint8((int32_t)(data->running.angle * 100), cmd);
      break;

    case RobotOpenCharge:
      cmd.push_back(9);
      cmd.push_back(data->task_type);
      cmd.push_back(data->charge.type);
      break;

    case RobotCross:
      cmd.push_back(11);
      cmd.push_back(data->task_type);
      cmd.push_back(data->cross.type);
      ConvertToUint8((int16_t)(data->cross.speed * 1000), cmd);
      ROS_INFO("robot cross start action ...");
      break;

    case RobotPutter:
      cmd.push_back(10);
      cmd.push_back(data->task_type);
      ConvertToUint8((uint16_t)(data->putter.hight), cmd);
      ROS_INFO("robot putter start action ...");
      break;

    default:
      return;
    }
    cmd.insert(cmd.begin() + 2, 0x00);
    cmd.insert(cmd.begin() + 3, 0x01);
    ConvertToUint8((uint16_t)Crc16Check(cmd, 0, cmd.size()), cmd);
    cmd.push_back(0x5a);
    SendData(cmd);
  }

  void SerialToStm32::DataDeal(uint8_t *data, size_t length)
  {
    for (int i = 0; i < length; i++)
    {
      if (data[i + 1] - 1 < length)
      {
        if (data[i] == 0xfc && data[data[i + 1] - 1 + i] == 0x5a)
        {
          std::vector<uint8_t> cmd;
          for (int j = 0; j < data[i + 1]; j++)
          {
            cmd.push_back(data[i + j]);
          }
          {
            boost::unique_lock<boost::shared_mutex> lockDeal(mutexDeal);
            data_cmd.push(cmd);
          }
          i = i + data[i + 1] - 1;
        }
      }
      else
        break;
    }
  }

  void SerialToStm32::CmdDeal(void)
  {
    ros::Time last_time = ros::Time::now();
    while (isRunning)
    {
      if (!data_cmd.empty())
      {
        boost::unique_lock<boost::shared_mutex> lockDeal(mutexDeal);
        std::vector<uint8_t> cmd = data_cmd.front();

        if (cmd[5] != 0)
          ROS_WARN("Task Can't Execute!");

        if (cmd[4] == RobotStatus)
        {
          robot_state_msgs::robot_state state;
          int index = 6;
          int32_t data_32, temp;
          uint16_t data_u16;
          state.current_task = cmd[index++];
          state.speed = Uint8ToConvert<int32_t>(cmd, index, data_32) / 1000.0f;
          state.radius = Uint8ToConvert<int32_t>(cmd, index, data_32) / 1000.0f;
          state.odom_x = Uint8ToConvert<int32_t>(cmd, index, data_32) / 1000.0f;
          state.odom_y = Uint8ToConvert<int32_t>(cmd, index, data_32) / 1000.0f;
          state.odom_angle = Uint8ToConvert<int32_t>(cmd, index, data_32) / 100.0f;
          state.odom_radius = Uint8ToConvert<int32_t>(cmd, index, data_32) / 1000.0f;
          state.odom_speed_x = Uint8ToConvert<int32_t>(cmd, index, data_32) / 1000.0f;
          state.odom_speed_y = Uint8ToConvert<int32_t>(cmd, index, data_32) / 1000.0f;
          state.battery_capacity = cmd[index++];
          state.battery_current = Uint8ToConvert<uint16_t>(cmd, index, data_u16);
          state.battery_voltage = Uint8ToConvert<uint16_t>(cmd, index, data_u16);
          state.warning_code = Uint8ToConvert<int32_t>(cmd, index, data_32);
          state.crossSensorValue = cmd[index++];
          state.ultrasonicValue = Uint8ToConvert<uint16_t>(cmd, index, data_u16);
          state.error_code = Uint8ToConvert<int32_t>(cmd, index, data_32);
          state.charge_state = cmd[index++];
          state.putter_height = Uint8ToConvert<uint16_t>(cmd, index, data_u16);
          state.header.stamp = ros::Time::now(); 
          pubRobotState.publish(state);
        }
        data_cmd.pop();
        last_time=ros::Time::now();
      }
      if((ros::Time::now()-last_time).toSec()>1){
         LOG_EVERY_N(WARNING, 500)<<"serial data timeout!";
      }
      ros::Duration(0.01).sleep();
    }
  }

  void SerialToStm32::StateGet(const ros::TimerEvent &event)
  {
    //  robot_state_msgs::data_to_stm32 state;
    //  state.task_type=1;
    if (download_state != NO_IDLE)
      return;

    std::vector<uint8_t> cmd;
    cmd.push_back(0xfc);
    cmd.push_back(8);
    cmd.push_back(0x01);
    cmd.insert(cmd.begin() + 2, 0x00);
    cmd.insert(cmd.begin() + 3, 0x01);
    ConvertToUint8((uint16_t)Crc16Check(cmd, 0, cmd.size()), cmd);
    cmd.push_back(0x5a);
    SendData(cmd);

    // pubStateGet.publish(state);
  }

  void SerialToStm32::DataGet(const ros::TimerEvent &event)
  {
    {
      boost::unique_lock<boost::shared_mutex> lockDeal(mutexRead);
      int length = rec_data.size();

      for (int i = 0; i < length; i++)
      {
        if (rec_data[i + 1] - 1 < length)
        {
          if (rec_data[i] == 0xfc && rec_data[rec_data[i + 1] - 1 + i] == 0x5a)
          {
            std::vector<uint8_t> cmd;
            for (int j = 0; j < rec_data[i + 1]; j++)
            {
              cmd.push_back(rec_data[i + j]);
            }
            {
              boost::unique_lock<boost::shared_mutex> lockDeal(mutexDeal);
              data_cmd.push(cmd);
            }

            InfoShow(rec_data, i);
            rec_data.erase(rec_data.begin(), rec_data.begin() + rec_data[i + 1] + i);
            i = 0;
            length = rec_data.size();
          }
        }
        else
        {
          //InfoShow(i);
          break;
        }
      }
    }
  }

  void SerialToStm32::InfoShow(std::vector<uint8_t> &data, size_t length)
  {

    std::vector<char> info;
    for (size_t m = 0; m < length; m++)
      info.push_back(rec_data[m]);
    if (length > 3)
    {
      std::string infoshow = info.data();
      LOG(INFO) << infoshow;
    }
  }

}
