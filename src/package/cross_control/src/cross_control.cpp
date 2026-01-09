#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <can_msgs/Frame.h>
#include <actionlib/server/simple_action_server.h>
#include <cross_control/CrossBeltAction.h>
#include <mutex>
#include <string>
#include <chrono>
#include <sstream>

// 核心枚举定义
enum CROSS_TYPE
{
  NO_LOAD = 0,
  UP_LOAD_RIGHT = 1,
  UP_LOAD_LEFT = 2,
  DOWN_LOAD_RIGHT = 3,
  DOWN_LOAD_LEFT = 4
};
enum NMT_COMMAND
{
  NMT_RESET_COMM = 0x01,
  NMT_RESET_NODE = 0x02,
  NMT_START_REMOTE = 0x01,
  NMT_STOP_REMOTE = 0x02
};
enum MOTOR_STATE
{
  MOTOR_UNINIT = 0,
  MOTOR_ONLINE = 1,
  MOTOR_OFFLINE = 2
};

// CANopen状态字掩码
#define SW_READY_TO_SWITCH_ON 0x01
#define SW_SWITCHED_ON 0x02
#define SW_OPERATION_ENABLED 0x04
#define SW_FAULT 0x08

class CrossBeltController
{
public:
  typedef actionlib::SimpleActionServer<cross_control::CrossBeltAction> Server;
  typedef std::chrono::steady_clock::time_point TimePoint;

  CrossBeltController(ros::NodeHandle &nh) : nh_(nh),
                                             as_(nh_, "cross_belt_action", boost::bind(&CrossBeltController::executeAction, this, _1), false),
                                             count_(0), setFlag_(false), isOk_(false), cross_state_(0), sensorState_(0),
                                             action_running_(false), motor_state_(MOTOR_UNINIT),
                                             last_can_recv_time_(std::chrono::steady_clock::now()),
                                             motor_node_id_(1), can_timeout_ms_(1000),
                                             motor_status_word_(0), sdo_response_received_(false)
  {
    // 参数读取与转换
    int tmp_vals[7] = {2, 1, 500, 0x601, 0x2000, 0x01, 2000}; // 默认值
    nh_.param<int>("default_stop_time", tmp_vals[0], tmp_vals[0]);
    nh_.param<int>("default_delay_time", tmp_vals[1], tmp_vals[1]);
    nh_.param<int>("default_set_speed", tmp_vals[2], tmp_vals[2]);
    nh_.param<int>("motor_sdo_cob_id", tmp_vals[3], tmp_vals[3]);
    nh_.param<int>("speed_sdo_index", tmp_vals[4], tmp_vals[4]);
    nh_.param<int>("speed_sdo_subindex", tmp_vals[5], tmp_vals[5]);
    nh_.param<int>("init_response_timeout_ms", tmp_vals[6], tmp_vals[6]);

    default_stop_time_ = tmp_vals[0];
    default_delay_time_ = tmp_vals[1];
    default_set_speed_ = tmp_vals[2];
    motor_sdo_cob_id_ = static_cast<uint32_t>(tmp_vals[3]);
    speed_sdo_index_ = static_cast<uint16_t>(tmp_vals[4]);
    speed_sdo_subindex_ = static_cast<uint8_t>(tmp_vals[5]);
    init_response_timeout_ms_ = tmp_vals[6];

    nh_.param<int>("motor_node_id", motor_node_id_, 1);
    nh_.param<int>("can_timeout_ms", can_timeout_ms_, 1000);

    // ROS话题初始化
    sensor_sub_ = nh_.subscribe("/relay_states", 10, &CrossBeltController::sensorStateCallback, this);
    can_recv_sub_ = nh_.subscribe("/received_can_message_cross", 10, &CrossBeltController::canMsgCallback, this);
    can_send_pub_ = nh_.advertise<can_msgs::Frame>("send_can_message_cross", 10);
    check_motor_timer_ = nh_.createTimer(ros::Duration(1), &CrossBeltController::checkMotorOnline, this);

    // 电机初始化
    motor_state_ = initMotor() ? MOTOR_ONLINE : MOTOR_OFFLINE;
    ROS_INFO("Motor init %s, stop_time=%ds, delay=%ds, speed=%d",
             motor_state_ == MOTOR_ONLINE ? "success" : "failed",
             default_stop_time_, default_delay_time_, default_set_speed_);

    as_.start();
    ROS_INFO("CrossBelt Action Server started");
  }

  // 传感器状态回调
  void sensorStateCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    sensorState_ = 0;
    if (msg->data.size() >= 3)
    {
      sensorState_ |= (msg->data[0] == 1) ? 0x01 : 0x00;
      sensorState_ |= (msg->data[2] == 1) ? 0x02 : 0x00;
    }
    else
    {
      ROS_WARN("Sensor data size too small: %lu", msg->data.size());
    }
  }

  // CAN消息回调（解析响应+更新时间）
  void canMsgCallback(const can_msgs::Frame::ConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(can_msg_mutex_);
    last_can_msg_ = *msg;
    last_can_recv_time_ = std::chrono::steady_clock::now();

    // 解析SDO响应和状态字
    if (msg->id == (0x580 + motor_node_id_) && msg->dlc == 8)
    {
      sdo_response_received_ = true;
      if (msg->data[1] == 0x41 && msg->data[2] == 0x60)
      {
        motor_status_word_ = (msg->data[5] << 8) | msg->data[4];
      }
    }
  }

  // 发送NMT指令（带响应等待）
  bool sendNMTCommand(NMT_COMMAND cmd)
  {
    can_msgs::Frame can_msg;
    can_msg.id = 0x000 + motor_node_id_;
    can_msg.dlc = 2;
    can_msg.data[0] = static_cast<uint8_t>(cmd);
    can_msg.data[1] = static_cast<uint8_t>(motor_node_id_);
    can_send_pub_.publish(can_msg);

    ROS_ERROR("NMT cmd 0x%02X ", cmd);
    return true;
  }

  // 发送SDO写指令
  bool sendSDOWrite(uint16_t index, uint8_t subindex, uint32_t data, bool wait_response = true)
  {
    sdo_response_received_ = false;
    can_msgs::Frame can_msg;
    can_msg.id = 0x600 + motor_node_id_;
    can_msg.dlc = 8;
    can_msg.data[0] = 0x23;
    can_msg.data[1] = index & 0xFF;
    can_msg.data[2] = (index >> 8) & 0xFF;
    can_msg.data[3] = subindex;
    can_msg.data[4] = data & 0xFF;
    can_msg.data[5] = (data >> 8) & 0xFF;
    can_msg.data[6] = (data >> 16) & 0xFF;
    can_msg.data[7] = (data >> 24) & 0xFF;
    can_send_pub_.publish(can_msg);

    if (!wait_response)
    {
      ros::Duration(0.05).sleep();
      return true;
    }

    // 等待响应
    TimePoint start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() < init_response_timeout_ms_)
    {
      if (sdo_response_received_)
        return true;
      ros::Duration(0.01).sleep();
      ros::spinOnce();
    }
    ROS_ERROR("SDO write 0x%X timeout", index);
    return false;
  }

  // 发送SDO读指令
  bool sendSDORead(uint16_t index, uint8_t subindex)
  {
    sdo_response_received_ = false;
    can_msgs::Frame can_msg;
    can_msg.id = 0x600 + motor_node_id_;
    can_msg.dlc = 8;
    can_msg.data[0] = 0x40;
    can_msg.data[1] = index & 0xFF;
    can_msg.data[2] = (index >> 8) & 0xFF;
    can_msg.data[3] = subindex;
    can_send_pub_.publish(can_msg);

    // 等待响应
    TimePoint start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() < init_response_timeout_ms_)
    {
      if (sdo_response_received_)
        return true;
      ros::Duration(0.01).sleep();
      ros::spinOnce();
    }
    ROS_ERROR("SDO read 0x%X timeout", index);
    return false;
  }

  // 检查电机运行状态
  bool checkMotorRunningState()
  {
    if (!sendSDORead(0x6041, 0x00))
      return false;

    bool ready = motor_status_word_ & SW_READY_TO_SWITCH_ON;
    bool on = motor_status_word_ & SW_SWITCHED_ON;
    bool enable = motor_status_word_ & SW_OPERATION_ENABLED;
    bool fault = motor_status_word_ & SW_FAULT;

    if (fault)
    {
      ROS_ERROR("Motor fault, status: 0x%04X", motor_status_word_);
      return false;
    }
    return (ready && on && enable);
  }

  // 电机初始化
  bool initMotor()
  {
    motor_status_word_ = 0;
    // 初始化流程：复位→启动→设模式→使能→运行→校验
    if (!sendNMTCommand(NMT_RESET_COMM))
      return false;
    if (!sendNMTCommand(NMT_START_REMOTE))
      return false;
    if (!sendSDOWrite(0x6060, 0x00, 0x03))
      return false;
    if (!sendSDOWrite(0x6040, 0x00, 0x00000006))
      return false;
    if (!sendSDOWrite(0x6040, 0x00, 0x00000007))
      return false;

    ros::Duration(0.2).sleep();
    return checkMotorRunningState();
  }

  // 电机掉线检测
  void checkMotorOnline(const ros::TimerEvent &e)
  {
    std::lock_guard<std::mutex> lock(can_msg_mutex_);
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - last_can_recv_time_).count();

    if (duration_ms > can_timeout_ms_)
    {
      if (!sendSDORead(0x6041, 0x00) || !checkMotorRunningState())
      {
        motor_state_ = MOTOR_OFFLINE;
        ROS_ERROR("Motor offline, timeout: %dms", static_cast<int>(duration_ms));

        if (action_running_)
          stopCurrentAction("Motor offline");
        attemptMotorRecovery(); // 无限恢复
      }
    }
  }

  // 电机恢复（无限重试）
  void attemptMotorRecovery()
  {
    while (motor_state_ == MOTOR_OFFLINE && ros::ok())
    {
      if (initMotor())
      {
        motor_state_ = MOTOR_ONLINE;
        ROS_INFO("Motor recovery success");
        return;
      }
      ROS_ERROR("Recovery failed, retry in 2s");
      ros::Duration(2).sleep();
    }
  }

  // 停止当前动作
  void stopCurrentAction(const std::string &reason)
  {
    if (!action_running_)
      return;

    sendSDOWrite(0x6040, 0x00, 0x00000000, false);
    setMotorSpeedViaSDO(0);

    cross_control::CrossBeltResult result;
    result.success = false;
    result.message = "Action stopped: " + reason;
    as_.setAborted(result);

    action_running_ = false;
    resetState();
    ROS_WARN("Action stopped: %s", reason.c_str());
  }

  // 设置电机速度
  bool setMotorSpeedViaSDO(int16_t speed)
  {
    if (motor_state_ != MOTOR_ONLINE)
    {
      ROS_ERROR("Motor offline, can't set speed");
      return false;
    }

    sdo_response_received_ = false;
    can_msgs::Frame can_msg;
    can_msg.id = motor_sdo_cob_id_;
    can_msg.dlc = 8;
    can_msg.data[0] = 0x23;
    can_msg.data[1] = 0xFF;
    can_msg.data[2] = 0x60;
    can_msg.data[3] = 0x00;
    int32_t speed_data = static_cast<int32_t>(speed);
    can_msg.data[4] = speed_data & 0xFF;
    can_msg.data[5] = (speed_data >> 8) & 0xFF;
    can_msg.data[6] = (speed_data >> 16) & 0xFF;
    can_msg.data[7] = (speed_data >> 24) & 0xFF;
    can_send_pub_.publish(can_msg);

    // 等待响应（超时不中断）
    TimePoint start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() < init_response_timeout_ms_ / 2)
    {
      if (sdo_response_received_)
        return true;
      ros::Duration(0.01).sleep();
      ros::spinOnce();
    }
    ROS_WARN("Speed set timeout, speed: %d", speed);
    return true;
  }

  // 上下料核心逻辑
  bool crossControl(CROSS_TYPE type)
  {
    uint8_t sensor_type = 0;
    int speed = 0;

    // 确定传感器类型和速度方向
    if (type == UP_LOAD_RIGHT)
    {
      sensor_type = 0x01;
      speed = default_set_speed_;
    }
    else if (type == UP_LOAD_LEFT)
    {
      sensor_type = 0x02;
      speed = -default_set_speed_;
    }
    else if (type == DOWN_LOAD_RIGHT)
    {
      sensor_type = 0x02;
      speed = default_set_speed_;
    }
    else if (type == DOWN_LOAD_LEFT)
    {
      sensor_type = 0x01;
      speed = -default_set_speed_;
    }

    if (count_ < default_stop_time_ * 25 && !isOk_)
    {
      if (!setFlag_)
      {
        setMotorSpeedViaSDO(speed);
        setFlag_ = true;
      }
      else
      {
        count_++;
        setMotorSpeedViaSDO(speed);
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        isOk_ = (sensorState_ & sensor_type) ? true : false;
      }
      return false;
    }

    cross_state_ = (type == UP_LOAD_RIGHT || type == UP_LOAD_LEFT) ? 1 : 0;
    resetState();
    return true;
  }

  // 重置状态
  void resetState()
  {
    setFlag_ = false;
    count_ = 0;
    isOk_ = false;
  }

  // Action执行逻辑
  void executeAction(const cross_control::CrossBeltGoalConstPtr &goal)
  {
    // 前置检查
    if (motor_state_ != MOTOR_ONLINE)
    {
      cross_control::CrossBeltResult result;
      result.success = false;
      result.message = "Motor offline";
      as_.setAborted(result);
      return;
    }
    if (action_running_)
    {
      cross_control::CrossBeltResult result;
      result.success = false;
      result.message = "Action running";
      as_.setAborted(result);
      return;
    }

    action_running_ = true;
    resetState();
    CROSS_TYPE action_type = static_cast<CROSS_TYPE>(goal->cross_type);
    ROS_INFO("Start action: type=%d", action_type);

    ros::Rate loop_rate(25);
    bool sensor_ok = false;
    cross_control::CrossBeltFeedback feedback;
    cross_control::CrossBeltResult result;

    while (ros::ok() && action_running_)
    {
      // 异常检查
      if (motor_state_ != MOTOR_ONLINE)
      {
        stopCurrentAction("Motor offline");
        return;
      }
      if (as_.isPreemptRequested())
      {
        setMotorSpeedViaSDO(0);
        result.success = false;
        result.message = "Preempted";
        as_.setPreempted(result);
        action_running_ = false;
        return;
      }

      // 空动作直接返回
      if (action_type == NO_LOAD)
      {
        result.success = true;
        result.message = "No action";
        as_.setSucceeded(result);
        action_running_ = false;
        return;
      }

      // 执行上下料逻辑
      if (!sensor_ok)
      {
        feedback.status = "Waiting sensor";
        feedback.cross_state = cross_state_;
        feedback.is_ok = isOk_;
        as_.publishFeedback(feedback);

        if (crossControl(action_type))
          sensor_ok = true;
      }
      else
      {
        // 延迟停止
        feedback.status = "Delay phase";
        feedback.cross_state = cross_state_;
        feedback.is_ok = isOk_;
        as_.publishFeedback(feedback);

        count_++;
        if (count_ > default_delay_time_ * 25)
        {
          setMotorSpeedViaSDO(0);
          result.success = true;
          result.message = "Success";
          as_.setSucceeded(result);
          action_running_ = false;
          return;
        }
      }

      ros::spinOnce();
      loop_rate.sleep();
    }

    // 异常退出
    if (action_running_)
    {
      setMotorSpeedViaSDO(0);
      result.success = false;
      result.message = "Node shutdown";
      as_.setAborted(result);
      action_running_ = false;
    }
  }

private:
  // ROS核心对象
  ros::NodeHandle nh_;
  Server as_;
  ros::Subscriber sensor_sub_;
  ros::Subscriber can_recv_sub_;
  ros::Publisher can_send_pub_;
  ros::Timer check_motor_timer_;

  // 参数配置
  int default_stop_time_;
  int default_delay_time_;
  int default_set_speed_;
  uint32_t motor_sdo_cob_id_;
  uint16_t speed_sdo_index_;
  uint8_t speed_sdo_subindex_;
  int motor_node_id_;
  int can_timeout_ms_;
  int init_response_timeout_ms_;

  // 运行状态
  uint32_t count_;
  bool setFlag_;
  bool isOk_;
  uint8_t cross_state_;
  uint8_t sensorState_;
  bool action_running_;
  MOTOR_STATE motor_state_;
  TimePoint last_can_recv_time_;
  uint16_t motor_status_word_;
  bool sdo_response_received_;

  // 缓存与锁
  can_msgs::Frame last_can_msg_;
  std::mutex sensor_mutex_;
  std::mutex can_msg_mutex_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cross_control_node");
  ros::NodeHandle nh("~");

  try
  {
    CrossBeltController controller(nh);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    spinner.stop();
  }
  catch (const std::exception &e)
  {
    ROS_FATAL("Controller error: %s", e.what());
    return -1;
  }
  return 0;
}