#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h>
#include <pose_to_odom/PoseToOdomConfig.h>
#include <mutex>
#include <deque>
#include <numeric>
#include <cmath>

class PoseToOdomNode
{
public:
  PoseToOdomNode() : filter_window_size_(5),
                     max_position_jump_(0.5),
                     max_yaw_jump_(M_PI / 4), // 45度
                     last_pose_timestamp_(0.0),
                     last_x_(0.0), last_y_(0.0), last_yaw_(0.0)
  {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // 初始化参数
    private_nh.param<bool>("pub_tf", pub_tf, false);
    private_nh.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
    private_nh.param<std::string>("base_link_frame_id", base_link_frame_id_, "base_link");
    private_nh.param<double>("publish_rate", publish_rate_, 10.0);
    private_nh.param<int>("filter_window_size", filter_window_size_, 5);
    private_nh.param<double>("max_position_jump", max_position_jump_, 0.5);
    private_nh.param<double>("max_yaw_jump", max_yaw_jump_, M_PI / 4);

    // 动态参数服务器
    dynamic_reconfigure::Server<pose_to_odom::PoseToOdomConfig>::CallbackType f;
    f = boost::bind(&PoseToOdomNode::dynamicReconfigureCallback, this, _1, _2);
    dyn_rec_server_.setCallback(f);

    // 创建订阅者和发布者
    pose_sub_ = nh.subscribe("/tracked_pose", 10, &PoseToOdomNode::poseCallback, this);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odom_slam", 10);
    pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose_slam", 10);

    // 定时器控制发布频率
    timer_ = nh.createTimer(ros::Duration(1.0 / publish_rate_), &PoseToOdomNode::timerCallback, this);

    ROS_INFO("Pose to Odom node initialized with extended features");
  }

  // 动态参数回调
  void dynamicReconfigureCallback(pose_to_odom::PoseToOdomConfig &config, uint32_t level)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    filter_window_size_ = config.filter_window_size;
    max_position_jump_ = config.max_position_jump;
    max_yaw_jump_ = config.max_yaw_jump;
    ROS_INFO("Dynamic config updated: filter_window=%d, max_pos_jump=%.2f, max_yaw_jump=%.2f rad",
             filter_window_size_, max_position_jump_, max_yaw_jump_);
  }

  // 滑动平均滤波
  double applyFilter(std::deque<double> &buffer, double new_value)
  {
    buffer.push_back(new_value);
    if (buffer.size() > filter_window_size_)
      buffer.pop_front();

    double sum = std::accumulate(buffer.begin(), buffer.end(), 0.0);
    return sum / buffer.size();
  }

  // 异常检测
  bool isPoseValid(double x, double y, double yaw, double timestamp)
  {
    if (last_pose_timestamp_ == 0.0)
    {
      ROS_INFO("Receiving first valid pose data.");
      return true;
    }

    // 检查时间间隔
    double dt = timestamp - last_pose_timestamp_;
    if (dt <= 0 || dt > 1.0) // 时间戳异常或间隔过大
    {
      ROS_WARN("Invalid timestamp delta: %.2f. Discarding pose.", dt);
      return false;
    }

    // 检查位置跳变
    double dx = fabs(x - last_x_);
    double dy = fabs(y - last_y_);
    if (sqrt(dx * dx + dy * dy) > max_position_jump_)
    {
      ROS_WARN("Position jump detected: %.2f, %.2f -> %.2f, %.2f (distance: %.2f > %.2f)",
               last_x_, last_y_, x, y, sqrt(dx * dx + dy * dy), max_position_jump_);
      return false;
    }

    // 检查角度跳变
    double dyaw = fabs(remainder(yaw - last_yaw_, 2 * M_PI));
    if (dyaw > max_yaw_jump_)
    {
      ROS_WARN("Yaw jump detected: %.2f -> %.2f (delta: %.2f > %.2f)",
               last_yaw_, yaw, dyaw, max_yaw_jump_);
      return false;
    }

    return true;
  }

  // 速度计算
  // 速度计算 - 仅计算，不更新历史状态
  void calculateVelocity(double x, double y, double yaw, double timestamp)
  {
    double dt = timestamp - last_pose_timestamp_;
    if (dt <= 0)
    {
      linear_velocity_x_ = 0.0;
      linear_velocity_y_ = 0.0;
      angular_velocity_z_ = 0.0;
      return;
    }

    // 计算线速度
    double dx = x - last_x_;
    double dy = y - last_y_;
    linear_velocity_x_ = dx / dt;
    linear_velocity_y_ = dy / dt;

    // 计算角速度
    double dyaw = remainder(yaw - last_yaw_, 2 * M_PI);
    angular_velocity_z_ = dyaw / dt;
  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    geometry_msgs::PoseWithCovarianceStamped pose_slam;
    pose_slam.header = msg->header;
    pose_slam.pose.pose = msg->pose;
    pose_slam.pose.covariance[0] = 0.1;   // x
    pose_slam.pose.covariance[7] = 0.1;   // y
    pose_slam.pose.covariance[35] = 0.05; // yaw
    pose_pub_.publish(pose_slam);

    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double yaw = tf::getYaw(msg->pose.orientation);
    double timestamp = msg->header.stamp.toSec();

    // 异常检测
    if (!isPoseValid(x, y, yaw, timestamp))
    {
      // 如果数据无效，直接返回，不更新任何状态
      return;
    }

    // 应用滤波
    x_filtered_ = applyFilter(x_buffer_, x);
    y_filtered_ = applyFilter(y_buffer_, y);
    yaw_filtered_ = applyFilter(yaw_buffer_, yaw);

    // 计算速度（使用当前滤波后的数据和上一次的历史数据）
    calculateVelocity(x_filtered_, y_filtered_, yaw_filtered_, timestamp);

    // 【关键修复】只有在数据有效时，才更新历史数据
    // 将当前滤波后的数据保存为下一次计算的基准
    last_x_ = x_filtered_;
    last_y_ = y_filtered_;
    last_yaw_ = yaw_filtered_;
    last_pose_timestamp_ = timestamp;

    // 更新标志位，允许发布
    latest_pose_.header.stamp = msg->header.stamp;
    has_new_pose_ = true;
  }

  void timerCallback(const ros::TimerEvent &event)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_new_pose_)
    {
      ROS_WARN_THROTTLE(5.0, "No valid pose data received");
      return;
    }

    // 发布TF和Odometry
    if (pub_tf)
      publishTF(x_filtered_, y_filtered_, yaw_filtered_);
    publishOdometry(x_filtered_, y_filtered_, yaw_filtered_);

    has_new_pose_ = false;
  }

  void publishTF(double x, double y, double yaw)
  {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, 0.0));
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                          odom_frame_id_, base_link_frame_id_));
  }

  void publishOdometry(double x, double y, double yaw)
  {
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_link_frame_id_;

    // 位置
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    tf::quaternionTFToMsg(q, odom.pose.pose.orientation);

    // 速度
    odom.twist.twist.linear.x = linear_velocity_x_;
    odom.twist.twist.linear.y = linear_velocity_y_;
    odom.twist.twist.angular.z = angular_velocity_z_;

    // Covariance矩阵
    odom.pose.covariance[0] = 0.1;   // x
    odom.pose.covariance[7] = 0.1;   // y
    odom.pose.covariance[35] = 0.05; // yaw
    odom.twist.covariance[0] = 1;    // vx
    odom.twist.covariance[7] = 1;    // vy
    odom.twist.covariance[35] = 0.1; // vz

    odom_pub_.publish(odom);
  }

private:
  // ROS相关
  ros::Subscriber pose_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher pose_pub_;
  ros::Timer timer_;
  dynamic_reconfigure::Server<pose_to_odom::PoseToOdomConfig> dyn_rec_server_;

  // 配置参数
  bool pub_tf;
  std::string odom_frame_id_;
  std::string base_link_frame_id_;
  double publish_rate_;
  int filter_window_size_;
  double max_position_jump_;
  double max_yaw_jump_;

  // 数据存储
  geometry_msgs::PoseStamped latest_pose_;
  bool has_new_pose_ = false;
  std::mutex mutex_;

  // 滤波相关
  std::deque<double> x_buffer_;
  std::deque<double> y_buffer_;
  std::deque<double> yaw_buffer_;
  double x_filtered_ = 0.0;
  double y_filtered_ = 0.0;
  double yaw_filtered_ = 0.0;

  // 速度计算
  double linear_velocity_x_ = 0.0;
  double linear_velocity_y_ = 0.0;
  double angular_velocity_z_ = 0.0;

  // 历史数据
  double last_x_ = 0.0;
  double last_y_ = 0.0;
  double last_yaw_ = 0.0;
  double last_pose_timestamp_ = 0.0;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_to_odom_node");
  PoseToOdomNode node;
  ros::spin();
  return 0;
}