#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <tf/transform_listener.h>
#include <vector>
#include <string>
#include <mutex>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;

class PointCloudMerger
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher merged_cloud_pub_;
  ros::Timer publish_timer_;

  std::vector<ros::Subscriber> cloud_subscribers_;
  std::vector<sensor_msgs::PointCloud2::ConstPtr> latest_clouds_;
  std::mutex cloud_mutex_;

  tf::TransformListener tf_listener_;

  std::vector<std::string> input_topics_;
  std::string output_topic_;
  double publish_frequency_;
  bool use_latest_only_;
  bool transform_to_target_frame_;
  std::string target_frame_id_;

public:
  PointCloudMerger() : pnh_("~"), tf_listener_(ros::Duration(5.0))
  {
    loadParameters();

    merged_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 10, false);

    createSubscribers();

    publish_timer_ = nh_.createTimer(
        ros::Duration(1.0 / publish_frequency_),
        &PointCloudMerger::publishMergedCloud, this);

    ROS_INFO_STREAM("===== PointCloudMerger Initialized =====");
    ROS_INFO_STREAM("Input topics: " << input_topics_.size());
    for (const auto &topic : input_topics_)
      ROS_INFO_STREAM(" - " << topic);
    ROS_INFO_STREAM("Output topic: " << output_topic_);
    ROS_INFO_STREAM("Publish frequency: " << publish_frequency_ << "Hz");
    ROS_INFO_STREAM("Coordinate transform: " << std::boolalpha << transform_to_target_frame_);
    if (transform_to_target_frame_)
      ROS_INFO_STREAM("Target frame: " << target_frame_id_);
  }

  void loadParameters()
  {
    pnh_.param("publish_frequency", publish_frequency_, 10.0);
    pnh_.param("output_topic", output_topic_, std::string("/merged_pointcloud"));
    pnh_.param("use_latest_only", use_latest_only_, true);
    pnh_.param("transform_to_target_frame", transform_to_target_frame_, false);
    pnh_.param("target_frame_id", target_frame_id_, std::string("map"));

    if (!pnh_.getParam("input_topics", input_topics_) || input_topics_.empty())
    {
      ROS_ERROR("No input topics specified! Set 'input_topics' parameter");
      ros::shutdown();
      return;
    }

    latest_clouds_.resize(input_topics_.size());
  }

  void createSubscribers()
  {
    for (size_t i = 0; i < input_topics_.size(); ++i)
    {
      cloud_subscribers_.push_back(
          nh_.subscribe<sensor_msgs::PointCloud2>(
              input_topics_[i],
              5,
              boost::bind(&PointCloudMerger::cloudCallback, this, _1, i)));
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg, size_t index)
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    latest_clouds_[index] = cloud_msg;
  }

  bool transformPointCloud(const sensor_msgs::PointCloud2::ConstPtr &in_cloud,
                           sensor_msgs::PointCloud2 &out_cloud)
  {
    try
    {
      tf_listener_.waitForTransform(target_frame_id_,
                                    in_cloud->header.frame_id,
                                    in_cloud->header.stamp,
                                    ros::Duration(0.1));

      pcl_ros::transformPointCloud(target_frame_id_, *in_cloud, out_cloud, tf_listener_);
      return true;
    }
    catch (tf::TransformException &ex)
    {
      ROS_DEBUG_THROTTLE(5, "Transform failed %s -> %s: %s",
                         in_cloud->header.frame_id.c_str(),
                         target_frame_id_.c_str(),
                         ex.what());
      return false;
    }
  }

  void publishMergedCloud(const ros::TimerEvent &event)
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);

    bool has_data = false;
    for (const auto &cloud : latest_clouds_)
    {
      if (cloud)
      {
        has_data = true;
        break;
      }
    }

    if (!has_data)
    {
      ROS_DEBUG_THROTTLE(5, "No point cloud data received");
      return;
    }

    PointCloudXYZI merged_cloud;

    for (size_t i = 0; i < latest_clouds_.size(); ++i)
    {
      if (!latest_clouds_[i])
      {
        // ROS_WARN_THROTTLE(5, "No data for topic: %s", input_topics_[i].c_str());
        continue;
      }

      sensor_msgs::PointCloud2 temp_ros_cloud;
      bool transform_success = true;

      if (transform_to_target_frame_)
      {
        transform_success = transformPointCloud(latest_clouds_[i], temp_ros_cloud);
      }
      else
      {
        temp_ros_cloud = *latest_clouds_[i];
        target_frame_id_ = temp_ros_cloud.header.frame_id;
      }

      if (!transform_success)
        continue;

      PointCloudXYZI temp_cloud;
      pcl::fromROSMsg(temp_ros_cloud, temp_cloud);
      merged_cloud += temp_cloud;

      if (use_latest_only_)
        latest_clouds_[i].reset();
    }

    if (merged_cloud.empty())
    {
      ROS_WARN_THROTTLE(5, "Merged point cloud is empty");
      return;
    }

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(merged_cloud, output_msg);
    output_msg.header.stamp = ros::Time::now();
    output_msg.header.frame_id = target_frame_id_;

    if (merged_cloud_pub_.getNumSubscribers() > 0)
    {
      merged_cloud_pub_.publish(output_msg);
    }

    ROS_DEBUG_STREAM("Published merged cloud: " << merged_cloud.size() << " points (frame: " << target_frame_id_ << ")");
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_merger");

  try
  {
    PointCloudMerger merger;
    ros::spin();
  }
  catch (const std::exception &e)
  {
    ROS_FATAL("Exception: %s", e.what());
    return 1;
  }

  return 0;
}