#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/BatteryState.h>
#include <vector>
#include <iomanip>
#include <chrono>
#include <thread>

struct BMSData
{
  uint32_t battery_status_flag;
  float charge_current;
  float max_cell_voltage;
  float min_cell_voltage;
  float total_voltage;
  int8_t max_cell_temp;
  int8_t min_cell_temp;
  uint16_t cycle_count;
  float remaining_capacity;
  float total_capacity;
  uint8_t mosfet_status;
  bool checksum_valid;
};

class BMSParser
{
private:
  ros::NodeHandle nh_;
  ros::Publisher battery_pub_;
  serial::Serial ser_;
  std::vector<uint8_t> recv_buf_;
  float design_capacity_;
  
  // 串口重连参数
  std::string serial_port_;
  int serial_baudrate_;
  int reconnect_interval_;    // 重连间隔(秒)
  int max_reconnect_attempts_;// 最大重连次数，-1为无限重试
  bool serial_connected_;     // 串口连接状态

public:
  BMSParser() : serial_connected_(false)
  {
    battery_pub_ = nh_.advertise<sensor_msgs::BatteryState>("/battery_state", 10);
    
    // 读取ROS参数
    ros::NodeHandle nh_p("~");
    nh_p.param<std::string>("serial_port", serial_port_, "/dev/ttyUSB0");
    nh_p.param<int>("serial_baudrate", serial_baudrate_, 9600);
    nh_p.param<float>("design_capacity", design_capacity_, 26.0f);
    nh_p.param<int>("reconnect_interval", reconnect_interval_, 3);
    nh_p.param<int>("max_reconnect_attempts", max_reconnect_attempts_, -1);

    // 初始化串口
    connect_serial();
  }

  // 串口连接/重连函数
  bool connect_serial()
  {
    static int reconnect_attempts = 0;
    
    // 超过最大重试次数则停止重连
    if (max_reconnect_attempts_ > 0 && reconnect_attempts >= max_reconnect_attempts_)
    {
      ROS_FATAL("串口重连次数达到上限(%d)，停止重连", max_reconnect_attempts_);
      return false;
    }

    try
    {
      // 如果串口已打开，先关闭
      if (ser_.isOpen())
      {
        ser_.close();
      }

      ser_.setPort(serial_port_);
      ser_.setBaudrate(serial_baudrate_);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser_.setTimeout(to);
      ser_.open();

      serial_connected_ = true;
      reconnect_attempts = 0;
      ROS_INFO("串口 %s 连接成功，波特率：%d", serial_port_.c_str(), serial_baudrate_);
      return true;
    }
    catch (serial::IOException &e)
    {
      serial_connected_ = false;
      reconnect_attempts++;
      ROS_WARN("串口连接失败(%d/%d)：%s，%d秒后重试", 
               reconnect_attempts, 
               max_reconnect_attempts_ > 0 ? max_reconnect_attempts_ : 999,
               e.what(), 
               reconnect_interval_);
      std::this_thread::sleep_for(std::chrono::seconds(reconnect_interval_));
      return false;
    }
  }

  // 计算校验和（累加和补码）
  uint8_t calculate_checksum(const std::vector<uint8_t> &data, int start, int end)
  {
    uint16_t sum = 0;
    for (int i = start; i <= end; i++) sum += data[i];
    return (uint8_t)(0xFF - (sum & 0xFF) + 1);
  }

  // 解析BMS数据包
  BMSData parse_bms_packet(const std::vector<uint8_t> &packet)
  {
    BMSData data;
    data.checksum_valid = false;

    if (packet.size() != 32)
    {
      ROS_WARN("数据包长度错误，实际：%ld，期望：32", packet.size());
      return data;
    }

    // 校验和验证
    uint8_t received_checksum = packet.back();
    uint8_t calculated_checksum = calculate_checksum(packet, 0, packet.size() - 2);
    if (received_checksum != calculated_checksum)
    {
      ROS_WARN("校验和错误！接收：0x%02X，计算：0x%02X", received_checksum, calculated_checksum);
      return data;
    }
    data.checksum_valid = true;

    // 解析协议字段
    data.battery_status_flag = (packet[8] << 24) | (packet[7] << 16) | (packet[6] << 8) | packet[5];
    data.charge_current = ((packet[10] << 8) | packet[9]) * 0.01f;
    data.max_cell_voltage = (packet[12] << 8) | packet[11];
    data.min_cell_voltage = (packet[14] << 8) | packet[13];
    data.total_voltage = ((packet[16] << 8) | packet[15]) * 0.01f;
    data.max_cell_temp = packet[17];
    data.min_cell_temp = packet[18];
    data.cycle_count = (packet[20] << 8) | packet[19];
    data.remaining_capacity = ((packet[22] << 8) | packet[21]) * 0.1f;
    data.total_capacity = ((packet[24] << 8) | packet[23]) * 0.1f;
    data.mosfet_status = packet[25];

    return data;
  }

  // 查找完整数据包
  std::vector<uint8_t> find_complete_packet()
  {
    std::vector<uint8_t> packet;
    for (size_t i = 0; i < recv_buf_.size(); i++)
    {
      if (recv_buf_[i] == 0x7F)
      {
        if (i + 3 >= recv_buf_.size()) break;
        uint8_t len = recv_buf_[i + 3];
        size_t total_len = 4 + len + 1;
        if (i + total_len <= recv_buf_.size())
        {
          packet.assign(recv_buf_.begin() + i, recv_buf_.begin() + i + total_len);
          recv_buf_.erase(recv_buf_.begin(), recv_buf_.begin() + i + total_len);
          return packet;
        }
        else
        {
          recv_buf_.erase(recv_buf_.begin(), recv_buf_.begin() + i);
          break;
        }
      }
    }
    if (recv_buf_.size() > 1024) recv_buf_.erase(recv_buf_.begin(), recv_buf_.end() - 1024);
    return packet;
  }

  // 构建标准BatteryState消息
  sensor_msgs::BatteryState build_battery_msg(const BMSData &bms_data)
  {
    sensor_msgs::BatteryState msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "bms";

    // 核心参数映射
    msg.voltage = bms_data.total_voltage;
    msg.current = bms_data.charge_current;
    msg.charge = bms_data.remaining_capacity;
    msg.capacity = bms_data.total_capacity;
    msg.design_capacity = design_capacity_;
    msg.percentage = (bms_data.remaining_capacity / bms_data.total_capacity) * 100.0f;

    // 电池类型与状态
    msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    bool is_charging = (bms_data.charge_current > 0.01f) && ((bms_data.mosfet_status & 0x80) == 0x80);
    msg.power_supply_status = is_charging ? 
                              sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING :
                              sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;

    // 额外信息
    msg.cell_voltage.resize(2);
    msg.cell_voltage[0] = bms_data.max_cell_voltage / 1000.0f;
    msg.cell_voltage[1] = bms_data.min_cell_voltage / 1000.0f;
    msg.temperature = (bms_data.max_cell_temp + bms_data.min_cell_temp) / 2.0f;

    return msg;
  }

  // 主运行循环
  void run()
  {
    ros::Rate rate(10);
    while (ros::ok())
    {
      // 串口断开则尝试重连
      if (!serial_connected_)
      {
        connect_serial();
        rate.sleep();
        continue;
      }

      try
      {
        if (ser_.available())
        {
          std::vector<uint8_t> temp_buf(ser_.available());
          ser_.read(temp_buf.data(), temp_buf.size());
          recv_buf_.insert(recv_buf_.end(), temp_buf.begin(), temp_buf.end());

          // 解析所有完整数据包
          std::vector<uint8_t> packet = find_complete_packet();
          while (!packet.empty())
          {
            BMSData bms_data = parse_bms_packet(packet);
            if (bms_data.checksum_valid)
            {
              sensor_msgs::BatteryState battery_msg = build_battery_msg(bms_data);
              battery_pub_.publish(battery_msg);
              ROS_INFO_THROTTLE(1, "发布电池数据：电压=%.2fV 剩余=%.1f%% 电流=%.2fA",
                                battery_msg.voltage, battery_msg.percentage, battery_msg.current);
            }
            packet = find_complete_packet();
          }
        }
      }
      catch (serial::IOException &e)
      {
        serial_connected_ = false;
        ROS_ERROR("串口通信异常：%s，触发重连", e.what());
      }

      rate.sleep();
    }

    // 程序退出时关闭串口
    if (ser_.isOpen()) ser_.close();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bms_parser_node");
  BMSParser parser;
  parser.run();
  return 0;
}