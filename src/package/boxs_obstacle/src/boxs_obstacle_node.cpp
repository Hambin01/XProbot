/*
 Copyright 2025 Google LLC

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      https://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <cmath>
#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <chrono>
#include <algorithm>

namespace point_cloud_filter
{

  using PointT = pcl::PointXYZ;
  using PointCloudT = pcl::PointCloud<PointT>;
  using PointCloudPtr = PointCloudT::Ptr;
  using MarkerArrayConstPtr = visualization_msgs::MarkerArray::ConstPtr;

  struct Config
  {
    std::string filter_mode = "exclude";              // exclude/include
    std::string point_cloud_topic = "rslidar_points"; // 点云话题
    std::string marker_topic = "arm_markers";         // 预测Marker话题
    std::string fallback_marker_frame = "base_link";  // Marker坐标系不存在时的备用坐标系

    // XYZ轴独立距离控制（替换原单一max_distance）
    double x_min = -5.0; // X轴最小值
    double x_max = 5.0;  // X轴最大值
    double y_min = -5.0; // Y轴最小值
    double y_max = 5.0;  // Y轴最大值
    double z_min = -5.0; // Z轴最小值
    double z_max = 5.0;  // Z轴最大值

    double downsample_leaf_size = 0.05;    // 降采样体素大小(米)
    int denoise_k = 10;                    // 去噪近邻数
    double denoise_std_dev = 0.01;         // 去噪标准差阈值
    double cylinder_radius_offset = 0.05;  // 圆柱半径偏移（独立参数）
    double cylinder_height_offset = 0.05;  // 圆柱高度偏移（新增独立参数）
    double cuboid_size_offset = 0.03;      // 立方体尺寸偏移
    int queue_size = 10;                   // 消息队列大小
    double tf_timeout = 1.0;               // TF变换超时时间(秒)
    bool log_tf_info = true;               // 打印TF相关日志
    bool log_filter_stats = true;          // 打印过滤统计信息
    bool log_marker_matching = false;      // 预测Marker数量多，默认关闭
    bool skip_transparent_markers = false; // 是否跳过低透明度的预测Marker
    double min_marker_alpha = 0.1;         // 最小透明度阈值
  };

  class PointCloudFilterNode
  {
  public:
    PointCloudFilterNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : nh_(nh), pnh_(pnh), tf_buffer_(std::make_shared<tf2_ros::Buffer>(ros::Duration(10.0))),
          tf_listener_(*tf_buffer_)
    {
      loadConfig();
      initPublishers();
      initSubscribers();
      printConfig();

      ROS_INFO("PointCloudFilterNode initialized (fixed cylinder detection)");
    }

    ~PointCloudFilterNode()
    {
      ROS_INFO("PointCloudFilterNode shutting down");
    }

    void run()
    {
      ros::spin();
    }

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    Config config_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Publisher pub_filtered_;
    ros::Publisher pub_preprocessed_;
    ros::Publisher pub_original_markers_;
    ros::Publisher pub_cloud_in_marker_frame_;

    ros::Subscriber sub_point_cloud_;
    ros::Subscriber sub_markers_;

    MarkerArrayConstPtr markers_;
    mutable std::string marker_frame_;
    mutable std::mutex markers_mutex_;

    void loadConfig()
    {
      pnh_.param<std::string>("filter_mode", config_.filter_mode, "exclude");
      pnh_.param<std::string>("point_cloud_topic", config_.point_cloud_topic, "rslidar_points");
      pnh_.param<std::string>("marker_topic", config_.marker_topic, "arm_markers");
      pnh_.param<std::string>("fallback_marker_frame", config_.fallback_marker_frame, "base_link");

      pnh_.param<double>("x_min", config_.x_min, -5.0);
      pnh_.param<double>("x_max", config_.x_max, 5.0);
      pnh_.param<double>("y_min", config_.y_min, -5.0);
      pnh_.param<double>("y_max", config_.y_max, 5.0);
      pnh_.param<double>("z_min", config_.z_min, -5.0);
      pnh_.param<double>("z_max", config_.z_max, 5.0);
      pnh_.param<double>("downsample_leaf_size", config_.downsample_leaf_size, 0.02);
      pnh_.param<int>("denoise_k", config_.denoise_k, 10);
      pnh_.param<double>("denoise_std_dev", config_.denoise_std_dev, 0.01);
      pnh_.param<double>("cylinder_radius_offset", config_.cylinder_radius_offset, 0.0);
      pnh_.param<double>("cylinder_height_offset", config_.cylinder_height_offset, 0.00);
      pnh_.param<double>("cuboid_size_offset", config_.cuboid_size_offset, 0.00);
      pnh_.param<int>("queue_size", config_.queue_size, 2);
      pnh_.param<double>("tf_timeout", config_.tf_timeout, 1.0);
      pnh_.param<bool>("log_tf_info", config_.log_tf_info, false);
      pnh_.param<bool>("log_filter_stats", config_.log_filter_stats, false);
      pnh_.param<bool>("log_marker_matching", config_.log_marker_matching, false);
      pnh_.param<bool>("skip_transparent_markers", config_.skip_transparent_markers, false);
      pnh_.param<double>("min_marker_alpha", config_.min_marker_alpha, 0.01);
    }

    void initPublishers()
    {
      pub_filtered_ = nh_.advertise<sensor_msgs::PointCloud2>(
          ros::this_node::getName() + "/filtered_rslidar_points", config_.queue_size);

      pub_preprocessed_ = nh_.advertise<sensor_msgs::PointCloud2>(
          ros::this_node::getName() + "/preprocessed_rslidar_points", config_.queue_size);

      pub_original_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(
          ros::this_node::getName() + "/received_predict_markers", config_.queue_size);

      pub_cloud_in_marker_frame_ = nh_.advertise<sensor_msgs::PointCloud2>(
          ros::this_node::getName() + "/rslidar_points_in_marker_frame", config_.queue_size);
    }

    void initSubscribers()
    {
      sub_point_cloud_ = nh_.subscribe(
          config_.point_cloud_topic, config_.queue_size,
          &PointCloudFilterNode::pointCloudCallback, this);

      sub_markers_ = nh_.subscribe(
          config_.marker_topic, config_.queue_size,
          &PointCloudFilterNode::markersCallback, this);
    }

    void printConfig() const
    {
      ROS_INFO("========================================");
      ROS_INFO("      Fixed Cylinder Filter Config       ");
      ROS_INFO("========================================");
      ROS_INFO("Filter Mode:          %s", config_.filter_mode.c_str());
      ROS_INFO("X Range:              [%.2f, %.2f] m", config_.x_min, config_.x_max);
      ROS_INFO("Y Range:              [%.2f, %.2f] m", config_.y_min, config_.y_max);
      ROS_INFO("Z Range:              [%.2f, %.2f] m", config_.z_min, config_.z_max);
      ROS_INFO("Cylinder Radius Offset: %.2f m", config_.cylinder_radius_offset);
      ROS_INFO("Cylinder Height Offset: %.2f m", config_.cylinder_height_offset);
      ROS_INFO("Cuboid Size Offset:   %.2f m", config_.cuboid_size_offset);
      ROS_INFO("========================================");
    }

    std::string getMarkerFrame() const
    {
      std::lock_guard<std::mutex> lock(markers_mutex_);

      if (!marker_frame_.empty())
      {
        return marker_frame_;
      }

      if (markers_)
      {
        for (const auto &marker : markers_->markers)
        {
          if (marker.header.frame_id.empty() || marker.header.frame_id == "unknown" ||
              marker.action == visualization_msgs::Marker::DELETE ||
              marker.action == visualization_msgs::Marker::DELETEALL ||
              (config_.skip_transparent_markers && marker.color.a < config_.min_marker_alpha))
          {
            continue;
          }
          marker_frame_ = marker.header.frame_id;
          if (config_.log_tf_info)
          {
            ROS_INFO("Extracted marker frame: %s (ID: %d)",
                     marker_frame_.c_str(), marker.id);
          }
          return marker_frame_;
        }
      }

      ROS_WARN_THROTTLE(5.0, "No valid marker frame found, using fallback: %s",
                        config_.fallback_marker_frame.c_str());
      return config_.fallback_marker_frame;
    }

    bool transformCloudToMarkerFrame(const PointCloudPtr &input,
                                     PointCloudPtr &output,
                                     const std::string &cloud_frame,
                                     const ros::Time &stamp) const
    {
      if (!input || input->empty())
      {
        ROS_WARN("Empty input cloud for transformation");
        return false;
      }

      const std::string target_frame = getMarkerFrame();

      try
      {
        const geometry_msgs::TransformStamped transform =
            tf_buffer_->lookupTransform(target_frame,
                                        cloud_frame,
                                        ros::Time(0),
                                        ros::Duration(config_.tf_timeout));

        const Eigen::Affine3d eigen_transform = tf2::transformToEigen(transform);
        pcl::transformPointCloud(*input, *output, eigen_transform);

        if (config_.log_tf_info)
        {
          ROS_DEBUG_THROTTLE(1.0, "Transformed cloud to marker frame: %s", target_frame.c_str());
        }

        return true;
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN_THROTTLE(1.0, "TF error (cloud->marker): %s", ex.what());
        return false;
      }
    }

    void filterByDistance(PointCloudPtr &cloud) const
    {
      if (!cloud || cloud->empty())
        return;

      pcl::PassThrough<PointT> pass;
      PointCloudPtr temp_cloud(new PointCloudT);
      PointCloudPtr filtered_cloud(new PointCloudT);

      pass.setInputCloud(cloud);
      pass.setFilterFieldName("x");
      pass.setFilterLimits(config_.x_min, config_.x_max);
      pass.filter(*temp_cloud);

      pass.setInputCloud(temp_cloud);
      pass.setFilterFieldName("y");
      pass.setFilterLimits(config_.y_min, config_.y_max);
      pass.filter(*filtered_cloud);

      pass.setInputCloud(filtered_cloud);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(config_.z_min, config_.z_max);
      pass.filter(*temp_cloud);

      const size_t original_size = cloud->size();
      *cloud = *temp_cloud;

      ROS_DEBUG("Distance filter (XYZ): %lu -> %lu points", original_size, cloud->size());
    }

    void downsample(PointCloudPtr &cloud) const
    {
      if (!cloud || cloud->empty())
        return;

      pcl::VoxelGrid<PointT> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(config_.downsample_leaf_size,
                     config_.downsample_leaf_size,
                     config_.downsample_leaf_size);

      PointCloudPtr filtered(new PointCloudT);
      vg.filter(*filtered);

      const size_t original_size = cloud->size();
      *cloud = *filtered;

      ROS_DEBUG("Downsample: %lu -> %lu points", original_size, cloud->size());
    }

    void denoise(PointCloudPtr &cloud) const
    {
      if (!cloud || cloud->empty() || cloud->size() < (size_t)config_.denoise_k)
      {
        return;
      }

      pcl::StatisticalOutlierRemoval<PointT> sor;
      sor.setInputCloud(cloud);
      sor.setMeanK(config_.denoise_k);
      sor.setStddevMulThresh(config_.denoise_std_dev);

      PointCloudPtr filtered(new PointCloudT);
      sor.filter(*filtered);

      const size_t original_size = cloud->size();
      *cloud = *filtered;

      ROS_DEBUG("Denoise: %lu -> %lu points", original_size, cloud->size());
    }

    bool isPointInSinglePredictMarker(const PointT &point, const visualization_msgs::Marker &marker) const
    {
      if (marker.action == visualization_msgs::Marker::DELETE ||
          marker.action == visualization_msgs::Marker::DELETEALL ||
          marker.scale.x <= 0 || marker.scale.y <= 0 || marker.scale.z <= 0)
      {
        return false;
      }

      if (config_.skip_transparent_markers && marker.color.a < config_.min_marker_alpha)
      {
        return false;
      }

      if (marker.type == visualization_msgs::Marker::CYLINDER)
      {
        const double cyl_diameter = std::max(marker.scale.x, marker.scale.y);
        const double base_radius = cyl_diameter / 2.0;
        const double final_radius = base_radius + config_.cylinder_radius_offset;

        const double base_height = marker.scale.z;
        const double final_height = base_height + 2 * config_.cylinder_height_offset;

        Eigen::Affine3d marker_pose;
        tf2::fromMsg(marker.pose, marker_pose);
        Eigen::Affine3d marker_pose_inv = marker_pose.inverse();

        Eigen::Vector3d point_world(point.x, point.y, point.z);
        Eigen::Vector3d point_local = marker_pose_inv * point_world;

        const double dist_to_axis = std::sqrt(point_local.x() * point_local.x() +
                                              point_local.y() * point_local.y());
        const double z_min_local = -final_height / 2.0;
        const double z_max_local = final_height / 2.0;

        bool in_cylinder = (dist_to_axis <= final_radius) &&
                           (point_local.z() >= z_min_local) &&
                           (point_local.z() <= z_max_local);

        return in_cylinder;
      }

      else if (marker.type == visualization_msgs::Marker::CUBE)
      {
        Eigen::Affine3d marker_pose;
        tf2::fromMsg(marker.pose, marker_pose);
        Eigen::Affine3d marker_pose_inv = marker_pose.inverse();

        Eigen::Vector3d point_world(point.x, point.y, point.z);
        Eigen::Vector3d point_local = marker_pose_inv * point_world;

        const double half_x = (marker.scale.x / 2.0) + config_.cuboid_size_offset;
        const double half_y = (marker.scale.y / 2.0) + config_.cuboid_size_offset;
        const double half_z = (marker.scale.z / 2.0) + config_.cuboid_size_offset;

        const bool in_x = (point_local.x() >= -half_x) && (point_local.x() <= half_x);
        const bool in_y = (point_local.y() >= -half_y) && (point_local.y() <= half_y);
        const bool in_z = (point_local.z() >= -half_z) && (point_local.z() <= half_z);

        return in_x && in_y && in_z;
      }

      return false;
    }

    bool isPointInAnyPredictMarker(const PointT &point) const
    {
      std::lock_guard<std::mutex> lock(markers_mutex_);

      if (!markers_ || markers_->markers.empty())
      {
        return false;
      }

      for (const auto &marker : markers_->markers)
      {
        if (isPointInSinglePredictMarker(point, marker))
        {
          return true;
        }
      }

      return false;
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
      try
      {
        {
          std::lock_guard<std::mutex> lock(markers_mutex_);
          if (!markers_)
          {
            ROS_DEBUG_THROTTLE(1.0, "Waiting for predict marker data...");
            return;
          }
        }

        PointCloudPtr cloud_original(new PointCloudT);
        pcl::fromROSMsg(*msg, *cloud_original);

        if (cloud_original->empty())
        {
          ROS_WARN_THROTTLE(1.0, "Received empty point cloud");
          return;
        }

        PointCloudPtr cloud_preprocessed(new PointCloudT(*cloud_original));
        filterByDistance(cloud_preprocessed);
        if (cloud_preprocessed->empty())
        {
          ROS_WARN_THROTTLE(1.0, "No points left after distance filter");
          return;
        }
        downsample(cloud_preprocessed);
        denoise(cloud_preprocessed);

        sensor_msgs::PointCloud2 preprocessed_msg;
        pcl::toROSMsg(*cloud_preprocessed, preprocessed_msg);
        preprocessed_msg.header = msg->header;
        if (pub_preprocessed_.getNumSubscribers() > 0)
        {
          pub_preprocessed_.publish(preprocessed_msg);
        }

        PointCloudPtr cloud_in_marker_frame(new PointCloudT);
        if (!transformCloudToMarkerFrame(cloud_preprocessed,
                                         cloud_in_marker_frame,
                                         msg->header.frame_id,
                                         msg->header.stamp))
        {
          return;
        }

        sensor_msgs::PointCloud2 cloud_marker_frame_msg;
        pcl::toROSMsg(*cloud_in_marker_frame, cloud_marker_frame_msg);
        cloud_marker_frame_msg.header = msg->header;
        cloud_marker_frame_msg.header.frame_id = getMarkerFrame();
        if (pub_cloud_in_marker_frame_.getNumSubscribers() > 0)
        {
          pub_cloud_in_marker_frame_.publish(cloud_marker_frame_msg);
        }

        PointCloudPtr cloud_filtered(new PointCloudT);
        size_t in_marker_count = 0;
        size_t total_markers = 0;

        {
          std::lock_guard<std::mutex> lock(markers_mutex_);
          total_markers = markers_->markers.size();
        }

        for (const auto &point : *cloud_in_marker_frame)
        {
          bool keep_point = false;
          bool in_any_marker = isPointInAnyPredictMarker(point);

          if (in_any_marker)
          {
            in_marker_count++;
          }

          if (config_.filter_mode == "exclude")
          {
            keep_point = !in_any_marker;
          }
          else if (config_.filter_mode == "include")
          {
            keep_point = in_any_marker;
          }

          if (keep_point)
          {
            cloud_filtered->push_back(point);
          }
        }

        if (config_.log_filter_stats)
        {
          ROS_INFO_THROTTLE(1.0,
                            "Filter Stats [Mode: %s] - Total Points: %lu, In Marker: %lu, Output: %lu, Total Markers: %lu",
                            config_.filter_mode.c_str(),
                            cloud_in_marker_frame->size(),
                            in_marker_count,
                            cloud_filtered->size(),
                            total_markers);

          if (config_.filter_mode == "include" && cloud_filtered->empty())
          {
            ROS_WARN_THROTTLE(1.0,
                              "INCLUDE MODE: No points in predict markers! Check:\n"
                              "1. Cylinder offset (radius/height)\n"
                              "2. Marker rotation/position\n"
                              "3. Point-cloud to marker frame transform");
          }
        }

        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_filtered, output_msg);
        output_msg.header = msg->header;
        output_msg.header.frame_id = getMarkerFrame();
        if (pub_filtered_.getNumSubscribers() > 0)
        {
          pub_filtered_.publish(output_msg);
        }
      }
      catch (const std::exception &e)
      {
        ROS_ERROR("Point cloud processing error: %s", e.what());
      }
      catch (...)
      {
        ROS_ERROR("Unknown error in point cloud callback");
      }
    }

    void markersCallback(const MarkerArrayConstPtr &msg)
    {
      if (!msg)
      {
        ROS_WARN("Received null marker array");
        return;
      }

      std::lock_guard<std::mutex> lock(markers_mutex_);
      markers_ = msg;
      marker_frame_.clear();

      pub_original_markers_.publish(*msg);
    }
  };

} // namespace point_cloud_filter

int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "cylinder_fixed_point_cloud_filter");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try
  {
    point_cloud_filter::PointCloudFilterNode filter_node(nh, pnh);
    filter_node.run();
  }
  catch (const std::exception &e)
  {
    ROS_FATAL("Failed to create node: %s", e.what());
    return 1;
  }
  catch (...)
  {
    ROS_FATAL("Unknown error creating node");
    return 1;
  }

  return 0;
}