#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types_conversion.h>
#include <mutex>
#include <random>
#include <cmath>

using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudXYZPtr = PointCloudXYZ::Ptr;
using PointCloudXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>;
using PointCloudXYZRGBPtr = PointCloudXYZRGB::Ptr;

class ObstacleDetector
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber pc_sub_;
  ros::Subscriber cmd_vel_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher debug_pc_pub_;
  ros::Publisher sound_pub_;

  // 聚类参数
  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;

  // 防抖参数
  int obstacle_count_threshold_;
  int clear_count_threshold_;
  int current_obstacle_count_;
  int current_clear_count_;
  bool is_obstacle_detected_;

  // 距离分级减速参数
  double min_stop_distance_;    // 最小停止距离（小于此距离直接停止）
  double decel_start_distance_; // 减速起始距离（大于此距离不减速）
  geometry_msgs::Twist current_cmd_vel_;

  std::mutex mutex_;
  std::mt19937 rng_;

public:
  ObstacleDetector() : rng_(std::random_device{}())
  {
    ros::NodeHandle pnh("~");

    pnh.param<double>("cluster_tolerance", cluster_tolerance_, 0.3);
    pnh.param<int>("min_cluster_size", min_cluster_size_, 50);
    pnh.param<int>("max_cluster_size", max_cluster_size_, 10000);

    pnh.param<int>("obstacle_count_threshold", obstacle_count_threshold_, 3);
    pnh.param<int>("clear_count_threshold", clear_count_threshold_, 5);

    pnh.param<double>("min_stop_distance", min_stop_distance_, 0.5);
    pnh.param<double>("decel_start_distance", decel_start_distance_, 2.0);

    current_obstacle_count_ = 0;
    current_clear_count_ = 0;
    is_obstacle_detected_ = false;
    current_cmd_vel_.linear.x = 0.0;
    current_cmd_vel_.linear.y = 0.0;
    current_cmd_vel_.linear.z = 0.0;
    current_cmd_vel_.angular.x = 0.0;
    current_cmd_vel_.angular.y = 0.0;
    current_cmd_vel_.angular.z = 0.0;

    pc_sub_ = nh_.subscribe("/obstacle_points", 1, &ObstacleDetector::pointCloudCallback, this);
    cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, &ObstacleDetector::cmdVelCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_stop", 1);
    debug_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/obstacle_debug_points", 1, false);
    sound_pub_ = nh_.advertise<std_msgs::String>("/sound_play", 1);

    ROS_INFO("Obstacle Detector Node Initialized!");
  }

  void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_cmd_vel_ = *msg;
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &pc_msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    PointCloudXYZPtr cloud(new PointCloudXYZ);
    pcl::fromROSMsg(*pc_msg, *cloud);

    double closest_obstacle_dist = 1000.0;
    bool has_valid_obstacle = false;

    if (!cloud->empty())
    {
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud(cloud);

      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance(cluster_tolerance_);
      ec.setMinClusterSize(min_cluster_size_);
      ec.setMaxClusterSize(max_cluster_size_);
      ec.setSearchMethod(tree);
      ec.setInputCloud(cloud);
      ec.extract(cluster_indices);

      // 计算每个聚类的中心点距离，取最近值
      if (!cluster_indices.empty())
      {
        has_valid_obstacle = true;
        for (const auto &indices : cluster_indices)
        {
          // 计算聚类中心点
          float cx = 0.0, cy = 0.0, cz = 0.0;
          for (int idx : indices.indices)
          {
            cx += cloud->points[idx].x;
            cy += cloud->points[idx].y;
            cz += cloud->points[idx].z;
          }
          cx /= indices.indices.size();
          cy /= indices.indices.size();
          cz /= indices.indices.size();

          double dist = sqrt(cx * cx + cy * cy);
          if (dist < closest_obstacle_dist)
          {
            closest_obstacle_dist = dist;
          }
        }
      }

      // 发布调试点云
      if (debug_pc_pub_.getNumSubscribers() > 0)
      {
        publishDebugPointCloud(cloud, cluster_indices, pc_msg->header);
      }
    }

    updateObstacleState(has_valid_obstacle);
    publishGradedSpeedCommand(closest_obstacle_dist);
  }

  void publishDebugPointCloud(const PointCloudXYZPtr &cloud,
                              const std::vector<pcl::PointIndices> &cluster_indices,
                              const std_msgs::Header &header)
  {
    PointCloudXYZRGBPtr debug_cloud(new PointCloudXYZRGB);
    debug_cloud->resize(cloud->size());
    debug_cloud->header = cloud->header;

    for (size_t i = 0; i < cloud->size(); ++i)
    {
      debug_cloud->points[i].x = cloud->points[i].x;
      debug_cloud->points[i].y = cloud->points[i].y;
      debug_cloud->points[i].z = cloud->points[i].z;
      debug_cloud->points[i].r = 128;
      debug_cloud->points[i].g = 128;
      debug_cloud->points[i].b = 128;
    }

    std::uniform_int_distribution<int> dist(0, 255);
    for (size_t i = 0; i < cluster_indices.size(); ++i)
    {
      uint8_t r = dist(rng_);
      uint8_t g = dist(rng_);
      uint8_t b = dist(rng_);
      for (const int &idx : cluster_indices[i].indices)
      {
        debug_cloud->points[idx].r = r;
        debug_cloud->points[idx].g = g;
        debug_cloud->points[idx].b = b;
      }
    }

    sensor_msgs::PointCloud2 debug_msg;
    pcl::toROSMsg(*debug_cloud, debug_msg);
    debug_msg.header = header;
    debug_pc_pub_.publish(debug_msg);
  }

  void updateObstacleState(bool current_detection)
  {
    if (current_detection)
    {
      current_obstacle_count_++;
      current_clear_count_ = 0;
      if (current_obstacle_count_ >= obstacle_count_threshold_)
      {
        is_obstacle_detected_ = true;
      }
    }
    else
    {
      current_clear_count_++;
      current_obstacle_count_ = 0;
      if (current_clear_count_ >= clear_count_threshold_)
      {
        is_obstacle_detected_ = false;
      }
    }
  }

  void publishGradedSpeedCommand(double closest_dist)
  {
    geometry_msgs::Twist output_cmd = current_cmd_vel_;

    if (is_obstacle_detected_)
    {

      if (closest_dist < min_stop_distance_)
      {
        output_cmd.linear.x = 0.0;
        output_cmd.linear.y = 0.0;
        output_cmd.linear.z = 0.0;
        output_cmd.angular.x = 0.0;
        output_cmd.angular.y = 0.0;
        output_cmd.angular.z = 0.0;
        std_msgs::String obstacle_msg;
        obstacle_msg.data = "检测到障碍物，停止";
        sound_pub_.publish(obstacle_msg);
      }
      else if (closest_dist <= decel_start_distance_)
      {
        // 计算速度系数：(距离 - 停止距离) / (减速起始距离 - 停止距离)
        double speed_factor = (closest_dist - min_stop_distance_) / (decel_start_distance_ - min_stop_distance_);
        speed_factor = std::max(0.0, std::min(1.0, speed_factor));

        output_cmd.linear.x *= speed_factor;
        output_cmd.linear.y *= speed_factor;
        output_cmd.linear.z *= speed_factor;
        output_cmd.angular.z *= speed_factor;

        // 微小速度阈值
        const double vel_thresh = 0.01;
        if (fabs(output_cmd.linear.x) < vel_thresh)
          output_cmd.linear.x = 0.0;
        if (fabs(output_cmd.linear.y) < vel_thresh)
          output_cmd.linear.y = 0.0;
        if (fabs(output_cmd.angular.z) < vel_thresh)
          output_cmd.angular.z = 0.0;
      }
    }

    cmd_vel_pub_.publish(output_cmd);
  }

  ~ObstacleDetector()
  {
    ROS_INFO("Obstacle Detector Node Shutdown!");
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_detection_node");
  ObstacleDetector detector;
  ros::spin();
  return 0;
}