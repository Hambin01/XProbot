#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Header.h>
#include <socketcan_bridge/topic_to_socketcan.h>
#include <can_msgs/Frame.h>
#include <vector>
#include <map>
#include <string>
#include <sstream>
#include <algorithm>

class RelayControlNode
{
private:
  ros::NodeHandle nh_;

  ros::Subscriber multi_relay_sub_;
  ros::Subscriber set_node_id_sub_;
  ros::Subscriber set_baudrate_sub_;

  ros::Publisher can_pub_;
  ros::Publisher relay_states_pub_;

  ros::Subscriber can_sub_;

  uint8_t current_node_id_;
  uint32_t current_baudrate_;
  uint8_t di_channels_;
  uint8_t do_channels_;
  const uint16_t RELAY_OBJ_INDEX = 0x6001;
  const uint16_t CONFIG_INDEX = 0x6900;

  const std::map<uint32_t, uint16_t> baudrate_map_ = {
      {20000, 0x0001}, {50000, 0x0002}, {100000, 0x0003}, {125000, 0x0004}, {200000, 0x0005}, {250000, 0x0006}, {400000, 0x0007}, {500000, 0x0008}, {800000, 0x0009}, {1000000, 0x000B}};

  std::map<uint8_t, bool> relay_states_;
  ros::Timer state_sync_timer_;
  const double SYNC_INTERVAL = 0.5;
  const double OFFLINE_TIMEOUT = 3.0; // 3秒无反馈判定掉线
  ros::Time last_response_time_;      // 最后一次硬件反馈时间
  bool hardware_online_;              // 硬件在线标志

public:
  RelayControlNode()
  {
    initParamsFromLaunch();

    can_pub_ = nh_.advertise<can_msgs::Frame>("/send_can_message_relay", 1);
    relay_states_pub_ = nh_.advertise<std_msgs::Int32MultiArray>("/relay_states", 1);

    can_sub_ = nh_.subscribe("/received_can_message_relay", 2, &RelayControlNode::canMsgCallback, this, ros::TransportHints().tcpNoDelay());

    multi_relay_sub_ = nh_.subscribe("/multi_relay_control", 10, &RelayControlNode::multiRelayControlCallback, this);
    set_node_id_sub_ = nh_.subscribe("/set_module_node_id", 10, &RelayControlNode::setNodeIdCallback, this);
    set_baudrate_sub_ = nh_.subscribe("/set_module_baudrate", 10, &RelayControlNode::setBaudrateCallback, this);

    // 初始化掉线检测状态
    hardware_online_ = false;
    last_response_time_ = ros::Time::now() - ros::Duration(OFFLINE_TIMEOUT);

    for (uint8_t i = 1; i <= do_channels_; ++i)
      relay_states_[i] = false;

    state_sync_timer_ = nh_.createTimer(ros::Duration(SYNC_INTERVAL), &RelayControlNode::syncRelayStates, this);

    ROS_INFO("Relay control node initialized - Node ID: %d, Baudrate: %d bps, DI: %d channels, DO: %d channels",
             current_node_id_, current_baudrate_, di_channels_, do_channels_);
  }

  void initParamsFromLaunch()
  {
    ros::NodeHandle nh_p("~");
    int node_id = 1;
    if (!nh_p.getParam("node_id", node_id) || node_id < 1 || node_id > 255)
    {
      ROS_WARN("Invalid node ID from launch, using default: 1");
      node_id = 1;
    }
    current_node_id_ = static_cast<uint8_t>(node_id);

    int baudrate = 1000000;
    if (!nh_p.getParam("baudrate", baudrate) || baudrate_map_.find(static_cast<uint32_t>(baudrate)) == baudrate_map_.end())
    {
      ROS_WARN("Invalid baudrate from launch, using default: 1000000 bps");
      baudrate = 1000000;
    }
    current_baudrate_ = static_cast<uint32_t>(baudrate);

    int di_ch = 8;
    if (!nh_p.getParam("di_channels", di_ch) || di_ch < 1 || di_ch > 64)
    {
      ROS_WARN("Invalid DI channels from launch, using default: 8");
      di_ch = 8;
    }
    di_channels_ = static_cast<uint8_t>(di_ch);

    int do_ch = 8;
    if (!nh_p.getParam("do_channels", do_ch) || do_ch < 1 || do_ch > 64)
    {
      ROS_WARN("Invalid DO channels from launch, using default: 8");
      do_ch = 8;
    }
    do_channels_ = static_cast<uint8_t>(do_ch);
  }

  bool buildSDODownloadFrame(uint16_t index, uint8_t subindex, uint32_t data, can_msgs::Frame &frame)
  {
    if (subindex > 0xFF)
    {
      ROS_ERROR("SDO frame build failed: invalid subindex %d", subindex);
      return false;
    }
    frame.id = 0x23 + (current_node_id_ << 1);
    frame.is_extended = false;
    frame.is_rtr = false;
    frame.dlc = 8;
    frame.data[0] = 0x23;
    frame.data[1] = index & 0xFF;
    frame.data[2] = (index >> 8) & 0xFF;
    frame.data[3] = subindex;
    frame.data[4] = data & 0xFF;
    frame.data[5] = (data >> 8) & 0xFF;
    frame.data[6] = (data >> 16) & 0xFF;
    frame.data[7] = (data >> 24) & 0xFF;
    return true;
  }

  bool buildSDOUploadFrame(uint16_t index, uint8_t subindex, can_msgs::Frame &frame)
  {
    if (subindex > 0xFF)
    {
      ROS_ERROR("SDO frame build failed: invalid subindex %d", subindex);
      return false;
    }
    frame.id = 0x23 + (current_node_id_ << 1);
    frame.is_extended = false;
    frame.is_rtr = false;
    frame.dlc = 8;
    frame.data[0] = 0x43;
    frame.data[1] = index & 0xFF;
    frame.data[2] = (index >> 8) & 0xFF;
    frame.data[3] = subindex;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    return true;
  }

  bool parseChannelString(const std::string &str, std::vector<uint8_t> &channels)
  {
    std::stringstream ss(str);
    std::string ch_str;
    while (getline(ss, ch_str, ','))
    {
      try
      {
        int ch = std::stoi(ch_str);
        if (ch >= 1 && ch <= 64)
          channels.push_back(static_cast<uint8_t>(ch));
      }
      catch (...)
      {
        return false;
      }
    }
    return !channels.empty();
  }

  void multiRelayControlCallback(const std_msgs::String::ConstPtr msg)
  {
    if (!hardware_online_)
    {
      ROS_ERROR("Multi-relay control failed: hardware is offline");
      return;
    }

    std::string content = msg->data;
    size_t split_pos = content.find(';');
    if (split_pos == std::string::npos)
    {
      ROS_ERROR("Invalid control format (required: 'channels:1,2,3;enable:true/false')");
      return;
    }

    std::string channel_str = content.substr(10, split_pos - 10);
    std::string enable_str = content.substr(split_pos + 8);
    bool enable = (enable_str == "true");

    std::vector<uint8_t> channels;
    if (!parseChannelString(channel_str, channels))
    {
      ROS_ERROR("Invalid channel format (required: '1,2,3' without prefix)");
      return;
    }

    for (auto it = channels.begin(); it != channels.end();)
    {
      if (*it > do_channels_)
      {
        ROS_WARN("Ignored channel %d (exceeds configured DO channels: %d)", *it, do_channels_);
        it = channels.erase(it);
      }
      else
        ++it;
    }

    std::sort(channels.begin(), channels.end());
    channels.erase(std::unique(channels.begin(), channels.end()), channels.end());

    if (channels.empty())
    {
      ROS_ERROR("No valid channels (1-%d only)", do_channels_);
      return;
    }

    uint32_t data_ch1_32 = 0x00, data_ch33_64 = 0x00;
    for (uint8_t ch : channels)
    {
      if (ch <= 32)
        data_ch1_32 |= (enable ? 1 << (ch - 1) : 0);
      else
        data_ch33_64 |= (enable ? 1 << (ch - 33) : 0);
      relay_states_[ch] = enable;
    }

    can_msgs::Frame frame1, frame2;
    bool send_ok = true;
    if (buildSDODownloadFrame(RELAY_OBJ_INDEX, 0x01, data_ch1_32, frame1))
      can_pub_.publish(frame1);
    else
      send_ok = false;

    if (data_ch33_64 != 0x00 && do_channels_ > 32)
    {
      if (buildSDODownloadFrame(RELAY_OBJ_INDEX, 0x02, data_ch33_64, frame2))
        can_pub_.publish(frame2);
      else
        send_ok = false;
    }

    if (send_ok)
    {
      std::string ch_str = channelListToString(channels);
      ROS_INFO("Multi-relay control success: channels [%s] %s", ch_str.c_str(), enable ? "opened" : "closed");
    }
    else
    {
      ROS_ERROR("Multi-relay control failed: SDO frame build error");
    }
  }

  void setNodeIdCallback(const std_msgs::Int32::ConstPtr msg)
  {
    if (!hardware_online_)
    {
      ROS_ERROR("Set node ID failed: hardware is offline");
      return;
    }

    int new_node_id = msg->data;
    if (new_node_id < 1 || new_node_id > 255)
    {
      ROS_ERROR("Node ID out of range (1-255 only)");
      return;
    }

    can_msgs::Frame frame;
    if (!buildSDODownloadFrame(CONFIG_INDEX, 0x01, static_cast<uint8_t>(new_node_id), frame))
    {
      ROS_ERROR("Set node ID failed: SDO frame build error");
      return;
    }

    can_pub_.publish(frame);
    current_node_id_ = static_cast<uint8_t>(new_node_id);
    ROS_INFO("Set node ID success: %d (power off 5s then on to take effect)", current_node_id_);
  }

  void setBaudrateCallback(const std_msgs::Int32::ConstPtr msg)
  {
    if (!hardware_online_)
    {
      ROS_ERROR("Set baudrate failed: hardware is offline");
      return;
    }

    uint32_t new_baudrate = static_cast<uint32_t>(msg->data);
    auto baud_it = baudrate_map_.find(new_baudrate);

    if (baud_it == baudrate_map_.end())
    {
      std::string support_list = getSupportedBaudrateList();
      ROS_ERROR("Unsupported baudrate: %d (supported: %s)", new_baudrate, support_list.c_str());
      return;
    }

    can_msgs::Frame frame;
    if (!buildSDODownloadFrame(CONFIG_INDEX, 0x02, baud_it->second, frame))
    {
      ROS_ERROR("Set baudrate failed: SDO frame build error");
      return;
    }

    can_pub_.publish(frame);
    current_baudrate_ = new_baudrate;
    ROS_INFO("Set baudrate success: %dbps (power off 5s then on to take effect)", current_baudrate_);
  }

  void canMsgCallback(const can_msgs::Frame::ConstPtr msg)
  {
    if (msg->id != (0x60 + current_node_id_) || msg->is_extended || msg->is_rtr || msg->dlc != 8)
      return;

    // 收到硬件反馈，更新最后响应时间
    last_response_time_ = ros::Time::now();

    if (msg->data[0] != 0x60)
    {
      ROS_WARN("SDO response error: command byte 0x%02X", msg->data[0]);
      return;
    }

    uint16_t index = (msg->data[2] << 8) | msg->data[1];
    uint8_t subindex = msg->data[3];
    uint32_t data = (msg->data[7] << 24) | (msg->data[6] << 16) | (msg->data[5] << 8) | msg->data[4];

    if (index == RELAY_OBJ_INDEX)
    {
      uint8_t start_ch = (subindex == 0x01) ? 1 : 33;
      uint8_t end_ch = std::min(static_cast<uint8_t>(start_ch + 31), do_channels_);
      for (uint8_t i = 0; i < (end_ch - start_ch + 1); ++i)
      {
        uint8_t ch = start_ch + i;
        relay_states_[ch] = (data >> i) & 0x01;
      }
      ROS_DEBUG("Relay state synced: channels %d-%d", start_ch, end_ch);
    }
  }

  void syncRelayStates(const ros::TimerEvent &event)
  {
    // 判定硬件在线状态
    double duration_since_last_response = (ros::Time::now() - last_response_time_).toSec();

    if (duration_since_last_response <= OFFLINE_TIMEOUT)
    {
      hardware_online_ = true;
    }
    else
    {
      hardware_online_ = false;
      ROS_WARN_STREAM_THROTTLE(5, "Relay offline!");
      return;
    }

    // 在线时发送状态查询并发布结果
    can_msgs::Frame frame1, frame2;
    if (do_channels_ > 0)
    {
      if (buildSDOUploadFrame(RELAY_OBJ_INDEX, 0x01, frame1))
        can_pub_.publish(frame1);
      if (do_channels_ > 32 && buildSDOUploadFrame(RELAY_OBJ_INDEX, 0x02, frame2))
      {
        ros::Duration(0.05).sleep();
        can_pub_.publish(frame2);
      }
    }

    std_msgs::Int32MultiArray states_msg;
    states_msg.data.resize(do_channels_, 0);
    for (uint8_t i = 1; i <= do_channels_; ++i)
      states_msg.data[i - 1] = relay_states_[i] ? 1 : 0;
    relay_states_pub_.publish(states_msg);
  }

  std::string channelListToString(const std::vector<uint8_t> &channels)
  {
    std::string str;
    for (size_t i = 0; i < channels.size(); ++i)
    {
      str += std::to_string(channels[i]);
      if (i != channels.size() - 1)
        str += ",";
    }
    return str;
  }

  std::string getSupportedBaudrateList()
  {
    std::string list;
    for (const auto &baud : baudrate_map_)
      list += std::to_string(baud.first) + "bps,";
    if (!list.empty())
      list.pop_back();
    return list;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "relay_control_node", ros::init_options::AnonymousName);
  RelayControlNode node;
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  return 0;
}