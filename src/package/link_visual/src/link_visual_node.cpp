/*
 * @Author: hambin
 */
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <dynamic_reconfigure/server.h>
#include <link_visual/ArmVisualizerConfigConfigConfig.h>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

// 路径预测相关配置
struct PredictConfig
{
  bool enable_prediction = false;    // 是否启用路径预测
  double predict_duration = 2.0;     // 预测总时长（秒）
  double predict_step = 0.2;         // 预测步长（秒）
  int predict_point_count;           // 预测点数量（自动计算）
  geometry_msgs::Twist last_cmd_vel; // 最后接收的速度指令
  ros::Time last_cmd_vel_time;       // 最后接收速度的时间戳
};

namespace tf
{
  void quaternionFromEuler(double roll, double pitch, double yaw, geometry_msgs::Quaternion &quaternion)
  {
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    quaternion.w = cy * cp * cr + sy * sp * sr;
    quaternion.x = cy * cp * sr - sy * sp * cr;
    quaternion.y = sy * cp * sr + cy * sp * cr;
    quaternion.z = sy * cp * cr - cy * sp * sr;
  }

  void eulerFromQuaternion(const geometry_msgs::Quaternion &quaternion, double &roll, double &pitch, double &yaw)
  {
    double x = quaternion.x;
    double y = quaternion.y;
    double z = quaternion.z;
    double w = quaternion.w;

    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (w * y - z * x);
    if (fabs(sinp) >= 1)
      pitch = copysign(M_PI / 2, sinp);
    else
      pitch = asin(sinp);

    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);
  }
}

struct DefaultParams
{
  // 圆柱默认参数
  double default_cylinder_radius = 0.05;
  std::vector<double> default_cylinder_color = {1.0, 0.0, 0.0, 1.0};
  std::vector<double> default_cylinder_offset = {0.0, 0.0, 0.0}; // 圆柱默认偏移（自身坐标系）

  // 长方体默认参数
  std::vector<double> default_cuboid_offset = {0.0, 0.1, 0.0};
  std::vector<double> default_cuboid_size = {0.08, 0.08, 0.15};
  std::vector<double> default_cuboid_color = {0.0, 1.0, 0.0, 0.8};
  std::vector<double> default_cuboid_rotate = {0.0, 0.0, 0.0};
  std::string default_cuboid_rotate_order = "xyz"; // 关节坐标系下的旋转顺序
};

class ArmJointLinkVisualizer
{
public:
  ArmJointLinkVisualizer(ros::NodeHandle &nh) : nh_(nh), tf_buffer_(ros::Duration(10.0)), tf_listener_(tf_buffer_)
  {
    nh_.param<std::string>("arm_base_frame", arm_base_frame_, "base_link");
    std::vector<std::string> joint_frames;
    if (nh_.getParam("joint_frames", joint_frames))
    {
      joint_frames_ = joint_frames;
    }

    nh_.param<double>("update_frequency", update_frequency_, 10.0);
    nh_.param<double>("marker_lifetime", marker_lifetime_, 0.1);
    nh_.param<std::string>("config_path", config_path_, "");

    predict_config_.enable_prediction = false;
    nh_.param<bool>("enable_path_prediction", predict_config_.enable_prediction, false);
    nh_.param<double>("predict_duration", predict_config_.predict_duration, 2.0);
    nh_.param<double>("predict_step", predict_config_.predict_step, 0.2);
    predict_config_.predict_point_count = std::ceil(predict_config_.predict_duration / predict_config_.predict_step);
    predict_config_.last_cmd_vel_time = ros::Time::now();

    default_params_ = DefaultParams();
    loadConfigFromYaml();
    initParamLists();
    validateAllParams();

    arm_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(ros::this_node::getName() + "/arm_visual_markers", 10);
    predict_arm_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(ros::this_node::getName() + "/arm_visual_markers_predict", 10);

    cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &ArmJointLinkVisualizer::cmdVelCallback, this);

    dyn_reconf_server_.setCallback(boost::bind(&ArmJointLinkVisualizer::dynReconfCallback, this, _1, _2));

    printConfigInfo();
  }

  void run()
  {
    ros::Rate rate(update_frequency_);
    while (ros::ok())
    {
      updateMergedMarkers();
      if (predict_config_.enable_prediction)
      {
        updatePredictMergedMarkers();
      }
      rate.sleep();
      ros::spinOnce();
    }
  }

private:
  ros::NodeHandle nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Publisher arm_marker_pub_;
  ros::Publisher predict_arm_marker_pub_;
  ros::Subscriber cmd_vel_sub_;
  dynamic_reconfigure::Server<link_visual::ArmVisualizerConfigConfigConfig> dyn_reconf_server_;

  std::string arm_base_frame_;
  std::vector<std::string> joint_frames_;
  double update_frequency_;
  double marker_lifetime_;
  std::string config_path_;

  // 参数配置
  DefaultParams default_params_;
  std::vector<double> cylinder_radii_;
  std::vector<std::vector<double>> cylinder_colors_;
  std::vector<std::vector<double>> cylinder_offsets_; // 圆柱偏移（自身坐标系，x/y/z）
  std::vector<std::vector<double>> cuboid_offsets_;
  std::vector<std::vector<double>> cuboid_sizes_;
  std::vector<std::vector<double>> cuboid_colors_;
  std::vector<std::vector<double>> cuboid_rotates_;

  PredictConfig predict_config_;

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
  {
    predict_config_.last_cmd_vel = *msg;
    predict_config_.last_cmd_vel.linear.x= -predict_config_.last_cmd_vel.linear.x;
   // predict_config_.last_cmd_vel.angular.z= -predict_config_.last_cmd_vel.angular.z;
    predict_config_.last_cmd_vel_time = ros::Time::now();
    ROS_DEBUG("Received cmd_vel: linear=(%f, %f, %f), angular=(%f, %f, %f)",
              msg->linear.x, msg->linear.y, msg->linear.z,
              msg->angular.x, msg->angular.y, msg->angular.z);
  }

  geometry_msgs::Point calculatePredictOffset(int step_index)
  {
    geometry_msgs::Point offset;
    offset.x = 0.0;
    offset.y = 0.0;
    offset.z = 0.0;

    double time_step = step_index * predict_config_.predict_step;
    double time_since_last_cmd = (ros::Time::now() - predict_config_.last_cmd_vel_time).toSec();
    if (time_since_last_cmd > 2.0)
    {
      return offset;
    }

    offset.x = predict_config_.last_cmd_vel.linear.x * time_step;
    offset.y = predict_config_.last_cmd_vel.linear.y * time_step;
    offset.z = predict_config_.last_cmd_vel.linear.z * time_step;

    if (fabs(predict_config_.last_cmd_vel.angular.z) > 1e-6)
    {
      double yaw_angle = predict_config_.last_cmd_vel.angular.z * time_step;
      double cos_yaw = cos(yaw_angle);
      double sin_yaw = sin(yaw_angle);

      double temp_x = offset.x * cos_yaw - offset.y * sin_yaw;
      double temp_y = offset.x * sin_yaw + offset.y * cos_yaw;
      offset.x = temp_x;
      offset.y = temp_y;
    }

    return offset;
  }

  geometry_msgs::Quaternion calculatePredictRotation(const geometry_msgs::Quaternion &original_quat, int step_index)
  {
    geometry_msgs::Quaternion predict_quat = original_quat;
    double time_since_last_cmd = (ros::Time::now() - predict_config_.last_cmd_vel_time).toSec();

    if (time_since_last_cmd > 2.0 || fabs(predict_config_.last_cmd_vel.angular.z) < 1e-6)
    {
      return predict_quat;
    }

    double time_step = step_index * predict_config_.predict_step;
    double yaw_angle = predict_config_.last_cmd_vel.angular.z * time_step;

    double roll, pitch, yaw;
    tf::eulerFromQuaternion(original_quat, roll, pitch, yaw);

    yaw += yaw_angle;

    tf::quaternionFromEuler(roll, pitch, yaw, predict_quat);

    return predict_quat;
  }

  void offsetMarkerPosition(visualization_msgs::Marker &marker, const geometry_msgs::Point &offset)
  {
    marker.pose.position.x += offset.x;
    marker.pose.position.y += offset.y;
    marker.pose.position.z += offset.z;
  }

  // 生成合并的原始Marker
  void generateMergedOriginalMarkers(visualization_msgs::MarkerArray &merged_markers)
  {
    merged_markers.markers.clear();
    int global_marker_id = 0;

    // 添加长方体Marker
    for (int joint_idx = 0; joint_idx < joint_frames_.size(); ++joint_idx)
    {
      const std::string &joint_frame = joint_frames_[joint_idx];
      geometry_msgs::TransformStamped joint_trans;

      if (!getTransform(arm_base_frame_, joint_frame, joint_trans))
      {
        continue;
      }

      geometry_msgs::Point joint_point;
      joint_point.x = joint_trans.transform.translation.x;
      joint_point.y = joint_trans.transform.translation.y;
      joint_point.z = joint_trans.transform.translation.z;

      geometry_msgs::Quaternion joint_orientation = joint_trans.transform.rotation;

      visualization_msgs::Marker cuboid_marker = createJointCuboidMarker(joint_idx, joint_point, joint_orientation);
      cuboid_marker.ns = "arm_joint_cuboids";
      cuboid_marker.id = global_marker_id++;
      merged_markers.markers.push_back(cuboid_marker);
    }

    // 添加圆柱Marker
    for (int link_idx = 0; link_idx < (int)joint_frames_.size() - 1; ++link_idx)
    {
      const std::string &joint1_frame = joint_frames_[link_idx];
      const std::string &joint2_frame = joint_frames_[link_idx + 1];

      geometry_msgs::TransformStamped joint1_trans, joint2_trans;
      if (!getTransform(arm_base_frame_, joint1_frame, joint1_trans) ||
          !getTransform(arm_base_frame_, joint2_frame, joint2_trans))
      {
        continue;
      }

      geometry_msgs::Point joint1_point, joint2_point;
      joint1_point.x = joint1_trans.transform.translation.x;
      joint1_point.y = joint1_trans.transform.translation.y;
      joint1_point.z = joint1_trans.transform.translation.z;

      joint2_point.x = joint2_trans.transform.translation.x;
      joint2_point.y = joint2_trans.transform.translation.y;
      joint2_point.z = joint2_trans.transform.translation.z;

      visualization_msgs::Marker cylinder_marker = createCylinderMarker(link_idx, joint1_point, joint2_point);
      cylinder_marker.ns = "arm_link_cylinders";
      cylinder_marker.id = global_marker_id++;
      merged_markers.markers.push_back(cylinder_marker);
    }
  }

  // 更新合并后的原始Marker
  void updateMergedMarkers()
  {
    visualization_msgs::MarkerArray merged_markers;
    generateMergedOriginalMarkers(merged_markers);

    if (!merged_markers.markers.empty())
    {
      arm_marker_pub_.publish(merged_markers);
      ROS_DEBUG_STREAM("Published merged arm markers, total count: " << merged_markers.markers.size());
    }
  }

  void updatePredictMergedMarkers()
  {
    visualization_msgs::MarkerArray original_merged_markers;
    generateMergedOriginalMarkers(original_merged_markers);

    visualization_msgs::MarkerArray final_predict_markers;

    for (int step = 0; step < predict_config_.predict_point_count; step++)
    {
      geometry_msgs::Point offset = calculatePredictOffset(step);

      for (const auto &original_marker : original_merged_markers.markers)
      {
        visualization_msgs::Marker predict_marker = original_marker;
        predict_marker.lifetime = ros::Duration(marker_lifetime_);
        predict_marker.color.a = 0.2;

        offsetMarkerPosition(predict_marker, offset);
        predict_marker.pose.orientation = calculatePredictRotation(original_marker.pose.orientation, step);

        predict_marker.ns += "_predict_step_" + std::to_string(step);
        final_predict_markers.markers.push_back(predict_marker);
      }
    }

    if (!final_predict_markers.markers.empty())
    {
      predict_arm_marker_pub_.publish(final_predict_markers);
      ROS_DEBUG_STREAM("Published predict merged markers, total count: " << final_predict_markers.markers.size());
    }
  }

  void dynReconfCallback(link_visual::ArmVisualizerConfigConfigConfig &config, uint32_t level)
  {
    ROS_INFO("Received dynamic parameter update request");

    predict_config_.enable_prediction = config.enable_path_prediction;

    int operation_type = config.operation_type;
    int target_index = config.target_index;

    if (operation_type == 0)
    {
      int max_index = cylinder_radii_.size() - 1;
      if (target_index < 0 || target_index > max_index)
      {
        ROS_WARN_STREAM("Link index " << target_index << " out of range (0~" << max_index << "), ignoring update");
        return;
      }

      if (config.update_offset)
      {
        std::vector<double> new_offset = {config.offset_x, config.offset_y, config.offset_z};
        if (validateOffset(new_offset, "cylinder_offsets[" + std::to_string(target_index) + "]"))
        {
          cylinder_offsets_[target_index] = new_offset;
          ROS_INFO_STREAM("Link " << target_index << " cylinder offset updated to: "
                                  << new_offset[0] << "," << new_offset[1] << "," << new_offset[2]);
        }
      }
      config.offset_x = cylinder_offsets_[target_index][0];
      config.offset_y = cylinder_offsets_[target_index][1];
      config.offset_z = cylinder_offsets_[target_index][2];
    }
    else
    {
      int max_index = cuboid_offsets_.size() - 1;
      if (target_index < 0 || target_index > max_index)
      {
        ROS_WARN_STREAM("Joint index " << target_index << " out of range (0~" << max_index << "), ignoring update");
        return;
      }
    }

    if (operation_type == 0)
    {
      if (config.update_radius)
      {
        double new_radius = config.target_radius;
        if (validateRadius(new_radius, "cylinder_radii[" + std::to_string(target_index) + "]"))
        {
          cylinder_radii_[target_index] = new_radius;
          ROS_INFO_STREAM("Link " << target_index << " radius updated to: " << new_radius);
        }
      }

      if (config.update_color)
      {
        std::vector<double> new_color = {config.color_r, config.color_g, config.color_b, config.color_a};
        if (validateColor(new_color, "cylinder_colors[" + std::to_string(target_index) + "]"))
        {
          cylinder_colors_[target_index] = new_color;
          ROS_INFO_STREAM("Link " << target_index << " color updated to: "
                                  << new_color[0] << "," << new_color[1] << "," << new_color[2] << "," << new_color[3]);
        }
      }
      config.target_radius = cylinder_radii_[target_index];
      config.color_r = cylinder_colors_[target_index][0];
      config.color_g = cylinder_colors_[target_index][1];
      config.color_b = cylinder_colors_[target_index][2];
      config.color_a = cylinder_colors_[target_index][3];
    }
    else
    {
      if (config.update_offset)
      {
        std::vector<double> new_offset = {config.offset_x, config.offset_y, config.offset_z};
        if (validateOffset(new_offset, "cuboid_offsets[" + std::to_string(target_index) + "]"))
        {
          cuboid_offsets_[target_index] = new_offset;
          ROS_INFO_STREAM("Joint " << target_index << " offset updated to: "
                                   << new_offset[0] << "," << new_offset[1] << "," << new_offset[2]);
        }
      }

      if (config.update_size)
      {
        std::vector<double> new_size = {config.size_x, config.size_y, config.size_z};
        if (validateSize(new_size, "cuboid_sizes[" + std::to_string(target_index) + "]"))
        {
          cuboid_sizes_[target_index] = new_size;
          ROS_INFO_STREAM("Joint " << target_index << " size updated to: "
                                   << new_size[0] << "," << new_size[1] << "," << new_size[2]);
        }
      }

      if (config.update_color)
      {
        std::vector<double> new_color = {config.color_r, config.color_g, config.color_b, config.color_a};
        if (validateColor(new_color, "cuboid_colors[" + std::to_string(target_index) + "]"))
        {
          cuboid_colors_[target_index] = new_color;
          ROS_INFO_STREAM("Joint " << target_index << " color updated to: "
                                   << new_color[0] << "," << new_color[1] << "," << new_color[2] << "," << new_color[3]);
        }
      }

      if (config.update_rotate)
      {
        std::vector<double> new_rotate = {config.rotate_r, config.rotate_p, config.rotate_y};
        if (validateRotate(new_rotate, "cuboid_rotates[" + std::to_string(target_index) + "]"))
        {
          cuboid_rotates_[target_index] = new_rotate;
          ROS_INFO_STREAM("Joint " << target_index << " rotate updated to: "
                                   << new_rotate[0] << "," << new_rotate[1] << "," << new_rotate[2]);
        }
      }
      config.offset_x = cuboid_offsets_[target_index][0];
      config.offset_y = cuboid_offsets_[target_index][1];
      config.offset_z = cuboid_offsets_[target_index][2];
      config.size_x=cuboid_sizes_[target_index][0];
      config.size_y=cuboid_sizes_[target_index][1];
      config.size_z=cuboid_sizes_[target_index][2];
      config.color_r = cuboid_colors_[target_index][0];
      config.color_g = cuboid_colors_[target_index][1];
      config.color_b = cuboid_colors_[target_index][2];
      config.color_a = cuboid_colors_[target_index][3];
      config.rotate_r = cuboid_rotates_[target_index][0];
      config.rotate_p = cuboid_rotates_[target_index][1];
      config.rotate_y = cuboid_rotates_[target_index][2];
    }

    if (config.save_config && !config_path_.empty())
    {
      saveConfigToYaml(config_path_);
    }
    

    config.update_radius = false;
    config.update_offset = false;
    config.update_size = false;
    config.update_color = false;
    config.update_rotate = false;
    config.save_config = false;
  }

  // 初始化参数列表
  void initParamLists()
  {
    int link_count = joint_frames_.size() - 1;
    ROS_INFO_STREAM("initParamLists: link_count = " << link_count);

    // 圆柱参数初始化
    cylinder_radii_ = std::vector<double>(link_count, default_params_.default_cylinder_radius);
    cylinder_colors_ = std::vector<std::vector<double>>(link_count, default_params_.default_cylinder_color);
    cylinder_offsets_ = std::vector<std::vector<double>>(link_count, default_params_.default_cylinder_offset);

    std::vector<double> temp_radii;
    if (nh_.getParam("cylinder_radii", temp_radii))
    {
      cylinder_radii_ = padParamList(temp_radii, link_count, default_params_.default_cylinder_radius);
      ROS_INFO_STREAM("Loaded cylinder_radii from ROS param, size: " << cylinder_radii_.size());
    }
    else
    {
      ROS_WARN("cylinder_radii not found in ROS param, using default");
    }

    std::vector<std::vector<double>> temp_lists;
    if (loadParamListOfLists("cylinder_colors", temp_lists))
    {
      cylinder_colors_ = padParamListOfLists(temp_lists, link_count, default_params_.default_cylinder_color);
      ROS_INFO_STREAM("Loaded cylinder_colors from ROS param, size: " << cylinder_colors_.size());
    }
    else
    {
      ROS_WARN("cylinder_colors not found in ROS param, using default");
    }

    if (loadParamListOfLists("cylinder_offsets", temp_lists))
    {
      cylinder_offsets_ = padParamListOfLists(temp_lists, link_count, default_params_.default_cylinder_offset);
      ROS_INFO_STREAM("Loaded cylinder_offsets from ROS param, size: " << cylinder_offsets_.size());
    }
    else
    {
      ROS_WARN("cylinder_offsets not found in ROS param, using default");
    }

    int joint_count = joint_frames_.size();
    ROS_INFO_STREAM("initParamLists: joint_count = " << joint_count);

    // 长方体参数初始化
    cuboid_offsets_ = std::vector<std::vector<double>>(joint_count, default_params_.default_cuboid_offset);
    cuboid_sizes_ = std::vector<std::vector<double>>(joint_count, default_params_.default_cuboid_size);
    cuboid_colors_ = std::vector<std::vector<double>>(joint_count, default_params_.default_cuboid_color);
    cuboid_rotates_ = std::vector<std::vector<double>>(joint_count, default_params_.default_cuboid_rotate);

    if (loadParamListOfLists("cuboid_offsets", temp_lists))
    {
      cuboid_offsets_ = padParamListOfLists(temp_lists, joint_count, default_params_.default_cuboid_offset);
      ROS_INFO_STREAM("Loaded cuboid_offsets from ROS param, size: " << cuboid_offsets_.size());
    }
    else
    {
      ROS_WARN("cuboid_offsets not found in ROS param, using default");
    }

    if (loadParamListOfLists("cuboid_sizes", temp_lists))
    {
      cuboid_sizes_ = padParamListOfLists(temp_lists, joint_count, default_params_.default_cuboid_size);
      ROS_INFO_STREAM("Loaded cuboid_sizes from ROS param, size: " << cuboid_sizes_.size());
    }
    else
    {
      ROS_WARN("cuboid_sizes not found in ROS param, using default");
    }

    if (loadParamListOfLists("cuboid_colors", temp_lists))
    {
      cuboid_colors_ = padParamListOfLists(temp_lists, joint_count, default_params_.default_cuboid_color);
      ROS_INFO_STREAM("Loaded cuboid_colors from ROS param, size: " << cuboid_colors_.size());
    }
    else
    {
      ROS_WARN("cuboid_colors not found in ROS param, using default");
    }

    if (loadParamListOfLists("cuboid_rotates", temp_lists))
    {
      cuboid_rotates_ = padParamListOfLists(temp_lists, joint_count, default_params_.default_cuboid_rotate);
      ROS_INFO_STREAM("Loaded cuboid_rotates from ROS param, size: " << cuboid_rotates_.size());
    }
    else
    {
      ROS_WARN("cuboid_rotates not found in ROS param, using default");
    }
  }

  // 填充参数列表至目标长度
  std::vector<double> padParamList(const std::vector<double> &param_list, int target_length, double default_value)
  {
    std::vector<double> result = param_list;

    if (result.size() < target_length)
    {
      result.insert(result.end(), target_length - result.size(), default_value);
    }
    else if (result.size() > target_length)
    {
      result.resize(target_length);
    }

    return result;
  }

  // 填充二维参数列表至目标长度
  std::vector<std::vector<double>> padParamListOfLists(
      const std::vector<std::vector<double>> &param_list,
      int target_length,
      const std::vector<double> &default_value)
  {
    std::vector<std::vector<double>> result = param_list;

    if (result.size() < target_length)
    {
      result.insert(result.end(), target_length - result.size(), default_value);
    }
    else if (result.size() > target_length)
    {
      result.resize(target_length);
    }

    return result;
  }

  // 加载二维参数列表
  bool loadParamListOfLists(const std::string &param_name, std::vector<std::vector<double>> &out_list)
  {
    XmlRpc::XmlRpcValue param_value;
    if (!nh_.getParam(param_name, param_value))
    {
      return false;
    }

    if (param_value.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_WARN_STREAM("Parameter " << param_name << " is not an array");
      return false;
    }

    out_list.clear();
    for (int i = 0; i < param_value.size(); ++i)
    {
      if (param_value[i].getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_WARN_STREAM("Element " << i << " of " << param_name << " is not an array");
        continue;
      }

      std::vector<double> vec;
      for (int j = 0; j < param_value[i].size(); ++j)
      {
        if (param_value[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
            param_value[i][j].getType() == XmlRpc::XmlRpcValue::TypeInt)
        {
          vec.push_back(static_cast<double>(param_value[i][j]));
        }
      }
      out_list.push_back(vec);
    }

    return true;
  }

  void validateAllParams()
  {
    // 圆柱参数校验
    for (int i = 0; i < cylinder_radii_.size(); ++i)
    {
      if (!validateRadius(cylinder_radii_[i], "cylinder_radii[" + std::to_string(i) + "]"))
      {
        cylinder_radii_[i] = default_params_.default_cylinder_radius;
      }
    }

    for (int i = 0; i < cylinder_colors_.size(); ++i)
    {
      if (!validateColor(cylinder_colors_[i], "cylinder_colors[" + std::to_string(i) + "]"))
      {
        cylinder_colors_[i] = default_params_.default_cylinder_color;
      }
    }

    for (int i = 0; i < cylinder_offsets_.size(); ++i)
    {
      if (!validateOffset(cylinder_offsets_[i], "cylinder_offsets[" + std::to_string(i) + "]"))
      {
        cylinder_offsets_[i] = default_params_.default_cylinder_offset;
      }
    }

    // 长方体参数校验
    for (int i = 0; i < cuboid_offsets_.size(); ++i)
    {
      if (!validateOffset(cuboid_offsets_[i], "cuboid_offsets[" + std::to_string(i) + "]"))
      {
        cuboid_offsets_[i] = default_params_.default_cuboid_offset;
      }
    }

    for (int i = 0; i < cuboid_sizes_.size(); ++i)
    {
      if (!validateSize(cuboid_sizes_[i], "cuboid_sizes[" + std::to_string(i) + "]"))
      {
        cuboid_sizes_[i] = default_params_.default_cuboid_size;
      }
    }

    for (int i = 0; i < cuboid_colors_.size(); ++i)
    {
      if (!validateColor(cuboid_colors_[i], "cuboid_colors[" + std::to_string(i) + "]"))
      {
        cuboid_colors_[i] = default_params_.default_cuboid_color;
      }
    }

    for (int i = 0; i < cuboid_rotates_.size(); ++i)
    {
      if (!validateRotate(cuboid_rotates_[i], "cuboid_rotates[" + std::to_string(i) + "]"))
      {
        cuboid_rotates_[i] = default_params_.default_cuboid_rotate;
      }
    }
  }

  // 校验半径合法性
  bool validateRadius(double radius, const std::string &param_name)
  {
    if (radius <= 0)
    {
      ROS_WARN_STREAM(param_name << " radius must be > 0, current value: " << radius << ", using default");
      return false;
    }
    return true;
  }

  // 校验尺寸合法性
  bool validateSize(const std::vector<double> &size, const std::string &param_name)
  {
    if (size.size() != 3)
    {
      ROS_WARN_STREAM(param_name << " size must be 3-dimensional, current length: " << size.size() << ", using default");
      return false;
    }

    for (double dim : size)
    {
      if (dim <= 0)
      {
        ROS_WARN_STREAM(param_name << " size dimension must be > 0, current value: " << size[0] << "," << size[1] << "," << size[2] << ", using default");
        return false;
      }
    }
    return true;
  }

  // 校验偏移量合法性
  bool validateOffset(const std::vector<double> &offset, const std::string &param_name)
  {
    if (offset.size() != 3)
    {
      ROS_WARN_STREAM(param_name << " offset must be 3-dimensional, current length: " << offset.size() << ", using default");
      return false;
    }
    return true;
  }

  // 校验颜色合法性
  bool validateColor(const std::vector<double> &color, const std::string &param_name)
  {
    if (color.size() != 4)
    {
      ROS_WARN_STREAM(param_name << " color must be RGBA 4-dimensional, current length: " << color.size() << ", using default");
      return false;
    }

    for (int i = 0; i < 4; ++i)
    {
      if (color[i] < 0 || color[i] > 1)
      {
        ROS_WARN_STREAM(param_name << " color value must be between 0~1, current value: " << color[0] << "," << color[1] << "," << color[2] << "," << color[3] << ", using default");
        return false;
      }
    }
    return true;
  }

  // 校验旋转角合法性
  bool validateRotate(const std::vector<double> &rotate, const std::string &param_name)
  {
    if (rotate.size() != 3)
    {
      ROS_WARN_STREAM(param_name << " rotate must be 3-dimensional, current length: " << rotate.size() << ", using default");
      return false;
    }
    return true;
  }

  // 解析旋转顺序并生成四元数
  Eigen::Quaterniond getRotateQuaternion(double roll, double pitch, double yaw, const std::string &order)
  {
    if (order == "xyz")
    {
      return Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    }
    else if (order == "zyx")
    {
      return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    }
    else
    {
      ROS_WARN("Invalid rotate order: %s, using xyz", order.c_str());
      return getRotateQuaternion(roll, pitch, yaw, "xyz");
    }
  }

  void loadConfigFromYaml()
  {
    if (config_path_.empty() || !std::ifstream(config_path_).good())
    {
      ROS_INFO_STREAM("No valid YAML config file specified, using default/ROS parameters");
      return;
    }

    try
    {
      YAML::Node config = YAML::LoadFile(config_path_);

      if (config["default_params"])
      {
        YAML::Node default_params = config["default_params"];

        // 圆柱默认参数加载
        if (default_params["default_cylinder_radius"])
        {
          default_params_.default_cylinder_radius = default_params["default_cylinder_radius"].as<double>();
          nh_.setParam("default_cylinder_radius", default_params_.default_cylinder_radius);
          ROS_INFO_STREAM("Loaded default_cylinder_radius: " << default_params_.default_cylinder_radius);
        }

        if (default_params["default_cylinder_color"])
        {
          default_params_.default_cylinder_color = default_params["default_cylinder_color"].as<std::vector<double>>();
          nh_.setParam("default_cylinder_color", default_params_.default_cylinder_color);
          ROS_INFO_STREAM("Loaded default_cylinder_color: "
                          << default_params_.default_cylinder_color[0] << ","
                          << default_params_.default_cylinder_color[1] << ","
                          << default_params_.default_cylinder_color[2] << ","
                          << default_params_.default_cylinder_color[3]);
        }

        if (default_params["default_cylinder_offset"])
        {
          default_params_.default_cylinder_offset = default_params["default_cylinder_offset"].as<std::vector<double>>();
          nh_.setParam("default_cylinder_offset", default_params_.default_cylinder_offset);
          ROS_INFO_STREAM("Loaded default_cylinder_offset: "
                          << default_params_.default_cylinder_offset[0] << ","
                          << default_params_.default_cylinder_offset[1] << ","
                          << default_params_.default_cylinder_offset[2]);
        }

        // 长方体默认参数加载
        if (default_params["default_cuboid_offset"])
        {
          default_params_.default_cuboid_offset = default_params["default_cuboid_offset"].as<std::vector<double>>();
          nh_.setParam("default_cuboid_offset", default_params_.default_cuboid_offset);
          ROS_INFO_STREAM("Loaded default_cuboid_offset: "
                          << default_params_.default_cuboid_offset[0] << ","
                          << default_params_.default_cuboid_offset[1] << ","
                          << default_params_.default_cuboid_offset[2]);
        }

        if (default_params["default_cuboid_size"])
        {
          default_params_.default_cuboid_size = default_params["default_cuboid_size"].as<std::vector<double>>();
          nh_.setParam("default_cuboid_size", default_params_.default_cuboid_size);
          ROS_INFO_STREAM("Loaded default_cuboid_size: "
                          << default_params_.default_cuboid_size[0] << ","
                          << default_params_.default_cuboid_size[1] << ","
                          << default_params_.default_cuboid_size[2]);
        }

        if (default_params["default_cuboid_color"])
        {
          default_params_.default_cuboid_color = default_params["default_cuboid_color"].as<std::vector<double>>();
          nh_.setParam("default_cuboid_color", default_params_.default_cuboid_color);
          ROS_INFO_STREAM("Loaded default_cuboid_color: "
                          << default_params_.default_cuboid_color[0] << ","
                          << default_params_.default_cuboid_color[1] << ","
                          << default_params_.default_cuboid_color[2] << ","
                          << default_params_.default_cuboid_color[3]);
        }

        if (default_params["default_cuboid_rotate"])
        {
          default_params_.default_cuboid_rotate = default_params["default_cuboid_rotate"].as<std::vector<double>>();
          nh_.setParam("default_cuboid_rotate", default_params_.default_cuboid_rotate);
          ROS_INFO_STREAM("Loaded default_cuboid_rotate: "
                          << default_params_.default_cuboid_rotate[0] << ","
                          << default_params_.default_cuboid_rotate[1] << ","
                          << default_params_.default_cuboid_rotate[2]);
        }

        // 加载旋转顺序参数
        if (default_params["default_cuboid_rotate_order"])
        {
          default_params_.default_cuboid_rotate_order = default_params["default_cuboid_rotate_order"].as<std::string>();
          nh_.setParam("default_cuboid_rotate_order", default_params_.default_cuboid_rotate_order);
          ROS_INFO_STREAM("Loaded default_cuboid_rotate_order: " << default_params_.default_cuboid_rotate_order);
        }
      }

      // 圆柱参数加载
      if (config["cylinder_radii"])
      {
        std::vector<double> radii = config["cylinder_radii"].as<std::vector<double>>();
        nh_.setParam("cylinder_radii", radii);
        ROS_INFO_STREAM("Loaded cylinder_radii, count: " << radii.size());
      }

      if (config["cylinder_colors"])
      {
        std::vector<std::vector<double>> colors = config["cylinder_colors"].as<std::vector<std::vector<double>>>();
        XmlRpc::XmlRpcValue xml_colors;
        xml_colors.setSize(colors.size());
        for (int i = 0; i < colors.size(); ++i)
        {
          XmlRpc::XmlRpcValue xml_color;
          xml_color.setSize(colors[i].size());
          for (int j = 0; j < colors[i].size(); ++j)
          {
            xml_color[j] = colors[i][j];
          }
          xml_colors[i] = xml_color;
        }
        nh_.setParam("cylinder_colors", xml_colors);
        ROS_INFO_STREAM("Loaded cylinder_colors, count: " << colors.size());
      }

      if (config["cylinder_offsets"])
      {
        std::vector<std::vector<double>> offsets = config["cylinder_offsets"].as<std::vector<std::vector<double>>>();
        XmlRpc::XmlRpcValue xml_offsets;
        xml_offsets.setSize(offsets.size());
        for (int i = 0; i < offsets.size(); ++i)
        {
          XmlRpc::XmlRpcValue xml_offset;
          xml_offset.setSize(offsets[i].size());
          for (int j = 0; j < offsets[i].size(); ++j)
          {
            xml_offset[j] = offsets[i][j];
          }
          xml_offsets[i] = xml_offset;
        }
        nh_.setParam("cylinder_offsets", xml_offsets);
        ROS_INFO_STREAM("Loaded cylinder_offsets, count: " << offsets.size());
      }

      // 长方体参数加载
      if (config["cuboid_offsets"])
      {
        std::vector<std::vector<double>> offsets = config["cuboid_offsets"].as<std::vector<std::vector<double>>>();
        XmlRpc::XmlRpcValue xml_offsets;
        xml_offsets.setSize(offsets.size());
        for (int i = 0; i < offsets.size(); ++i)
        {
          XmlRpc::XmlRpcValue xml_offset;
          xml_offset.setSize(offsets[i].size());
          for (int j = 0; j < offsets[i].size(); ++j)
          {
            xml_offset[j] = offsets[i][j];
          }
          xml_offsets[i] = xml_offset;
        }
        nh_.setParam("cuboid_offsets", xml_offsets);
        ROS_INFO_STREAM("Loaded cuboid_offsets, count: " << offsets.size());
      }

      if (config["cuboid_sizes"])
      {
        std::vector<std::vector<double>> sizes = config["cuboid_sizes"].as<std::vector<std::vector<double>>>();
        XmlRpc::XmlRpcValue xml_sizes;
        xml_sizes.setSize(sizes.size());
        for (int i = 0; i < sizes.size(); ++i)
        {
          XmlRpc::XmlRpcValue xml_size;
          xml_size.setSize(sizes[i].size());
          for (int j = 0; j < sizes[i].size(); ++j)
          {
            xml_size[j] = sizes[i][j];
          }
          xml_sizes[i] = xml_size;
        }
        nh_.setParam("cuboid_sizes", xml_sizes);
        ROS_INFO_STREAM("Loaded cuboid_sizes, count: " << sizes.size());
      }

      if (config["cuboid_colors"])
      {
        std::vector<std::vector<double>> colors = config["cuboid_colors"].as<std::vector<std::vector<double>>>();
        XmlRpc::XmlRpcValue xml_colors;
        xml_colors.setSize(colors.size());
        for (int i = 0; i < colors.size(); ++i)
        {
          XmlRpc::XmlRpcValue xml_color;
          xml_color.setSize(colors[i].size());
          for (int j = 0; j < colors[i].size(); ++j)
          {
            xml_color[j] = colors[i][j];
          }
          xml_colors[i] = xml_color;
        }
        nh_.setParam("cuboid_colors", xml_colors);
        ROS_INFO_STREAM("Loaded cuboid_colors, count: " << colors.size());
      }

      if (config["cuboid_rotates"])
      {
        std::vector<std::vector<double>> rotates = config["cuboid_rotates"].as<std::vector<std::vector<double>>>();
        XmlRpc::XmlRpcValue xml_rotates;
        xml_rotates.setSize(rotates.size());
        for (int i = 0; i < rotates.size(); ++i)
        {
          XmlRpc::XmlRpcValue xml_rotate;
          xml_rotate.setSize(rotates[i].size());
          for (int j = 0; j < rotates[i].size(); ++j)
          {
            xml_rotate[j] = rotates[i][j];
          }
          xml_rotates[i] = xml_rotate;
        }
        nh_.setParam("cuboid_rotates", xml_rotates);
        ROS_INFO_STREAM("Loaded cuboid_rotates, count: " << rotates.size());
      }

      ROS_INFO_STREAM("Successfully loaded ALL config from " << config_path_);
    }
    catch (YAML::BadFile &e)
    {
      ROS_ERROR_STREAM("YAML file not found: " << e.what());
    }
    catch (YAML::ParserException &e)
    {
      ROS_ERROR_STREAM("YAML parse error: " << e.what());
    }
    catch (std::exception &e)
    {
      ROS_ERROR_STREAM("Failed to load YAML config: " << e.what() << ", using default parameters");
    }
  }

  bool saveConfigToYaml(const std::string &save_path)
  {
    try
    {
      YAML::Node config;

      // 保存默认参数
      config["default_params"]["default_cylinder_radius"] = default_params_.default_cylinder_radius;
      config["default_params"]["default_cylinder_color"] = default_params_.default_cylinder_color;
      config["default_params"]["default_cylinder_offset"] = default_params_.default_cylinder_offset;
      config["default_params"]["default_cuboid_offset"] = default_params_.default_cuboid_offset;
      config["default_params"]["default_cuboid_size"] = default_params_.default_cuboid_size;
      config["default_params"]["default_cuboid_color"] = default_params_.default_cuboid_color;
      config["default_params"]["default_cuboid_rotate"] = default_params_.default_cuboid_rotate;
      config["default_params"]["default_cuboid_rotate_order"] = default_params_.default_cuboid_rotate_order;

      // 保存圆柱参数
      config["cylinder_radii"] = cylinder_radii_;
      config["cylinder_colors"] = cylinder_colors_;
      config["cylinder_offsets"] = cylinder_offsets_;

      // 保存长方体参数
      config["cuboid_offsets"] = cuboid_offsets_;
      config["cuboid_sizes"] = cuboid_sizes_;
      config["cuboid_colors"] = cuboid_colors_;
      config["cuboid_rotates"] = cuboid_rotates_;

      std::ofstream fout(save_path);
      fout << config;
      fout.close();

      ROS_INFO_STREAM("Config saved to " << save_path);
      return true;
    }
    catch (std::exception &e)
    {
      ROS_ERROR_STREAM("Failed to save config: " << e.what());
      return false;
    }
  }

  void printConfigInfo()
  {
    ROS_INFO("==============================================================");
    ROS_INFO("Arm Joint/Link Visualizer - Configuration Info");
    ROS_INFO_STREAM("Joint frames: ");
    for (const auto &frame : joint_frames_)
    {
      ROS_INFO_STREAM("  - " << frame);
    }
    ROS_INFO_STREAM("Number of joints: " << joint_frames_.size());
    ROS_INFO_STREAM("Number of links: " << (joint_frames_.size() - 1));
    ROS_INFO_STREAM("Default cylinder radius: " << default_params_.default_cylinder_radius);
    ROS_INFO_STREAM("Default cylinder offset: " << default_params_.default_cylinder_offset[0]
                                                << "," << default_params_.default_cylinder_offset[1] << "," << default_params_.default_cylinder_offset[2]);
    ROS_INFO_STREAM("Default cuboid offset: " << default_params_.default_cuboid_offset[0]
                                              << "," << default_params_.default_cuboid_offset[1] << "," << default_params_.default_cuboid_offset[2]);
    ROS_INFO_STREAM("Default cuboid size: " << default_params_.default_cuboid_size[0]
                                            << "," << default_params_.default_cuboid_size[1] << "," << default_params_.default_cuboid_size[2]);
    ROS_INFO_STREAM("Default cuboid rotate order: " << default_params_.default_cuboid_rotate_order);
    ROS_INFO_STREAM("Config file path: " << (config_path_.empty() ? "Not specified" : config_path_));

    ROS_INFO_STREAM("Path prediction enabled: " << (predict_config_.enable_prediction ? "Yes" : "No"));
    ROS_INFO_STREAM("Predict duration: " << predict_config_.predict_duration << "s");
    ROS_INFO_STREAM("Predict step: " << predict_config_.predict_step << "s");
    ROS_INFO_STREAM("Predict point count: " << predict_config_.predict_point_count);
    ROS_INFO_STREAM("Merged marker topic: /arm_visual_markers");
    ROS_INFO_STREAM("Predict merged marker topic: /arm_visual_markers_predict");
    ROS_INFO("==============================================================");
  }

  // 获取TF变换
  bool getTransform(const std::string &target_frame, const std::string &source_frame, geometry_msgs::TransformStamped &transform)
  {
    try
    {
      transform = tf_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(0.5));
      return true;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN_STREAM("Failed to get TF transform " << source_frame << " -> " << target_frame << ": " << ex.what());
      return false;
    }
  }

  // 创建圆柱Marker（增加自身坐标系偏移，Z轴为高度方向）
  visualization_msgs::Marker createCylinderMarker(int link_idx, const geometry_msgs::Point &joint1_point, const geometry_msgs::Point &joint2_point)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = arm_base_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "arm_link_cylinders";
    marker.id = link_idx;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(marker_lifetime_);

    marker.pose.position.x = (joint1_point.x + joint2_point.x) / 2.0;
    marker.pose.position.y = (joint1_point.y + joint2_point.y) / 2.0;
    marker.pose.position.z = (joint1_point.z + joint2_point.z) / 2.0;

    double axis_x = joint2_point.x - joint1_point.x;
    double axis_y = joint2_point.y - joint1_point.y;
    double axis_z = joint2_point.z - joint1_point.z;
    double length = sqrt(axis_x * axis_x + axis_y * axis_y + axis_z * axis_z);
    length = std::max(length, 1e-6);

    Eigen::Vector3d z_axis(axis_x / length, axis_y / length, axis_z / length);
    Eigen::Vector3d x_axis(1, 0, 0);

    if (fabs(z_axis.dot(x_axis)) > 0.999)
    {
      x_axis = Eigen::Vector3d(0, 1, 0);
    }

    Eigen::Vector3d y_axis = z_axis.cross(x_axis);
    y_axis.normalize();
    x_axis = y_axis.cross(z_axis);

    Eigen::Matrix3d rot_mat;
    rot_mat << x_axis.x(), y_axis.x(), z_axis.x(),
        x_axis.y(), y_axis.y(), z_axis.y(),
        x_axis.z(), y_axis.z(), z_axis.z();

    Eigen::Quaterniond quat(rot_mat);
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();

    std::vector<double> cyl_offset = cylinder_offsets_[link_idx];
    Eigen::Vector3d offset_local(cyl_offset[0], cyl_offset[1], cyl_offset[2]);
    Eigen::Vector3d offset_world = quat * offset_local;
    marker.pose.position.x += offset_world.x();
    marker.pose.position.y += offset_world.y();
    marker.pose.position.z += offset_world.z();

    marker.scale.x = cylinder_radii_[link_idx];
    marker.scale.y = cylinder_radii_[link_idx];
    marker.scale.z = length;
    marker.color.r = cylinder_colors_[link_idx][0];
    marker.color.g = cylinder_colors_[link_idx][1];
    marker.color.b = cylinder_colors_[link_idx][2];
    marker.color.a = cylinder_colors_[link_idx][3];

    return marker;
  }

  // 创建长方体Marker（修改核心：基于关节坐标系的偏移和旋转）
  visualization_msgs::Marker createJointCuboidMarker(int joint_idx, const geometry_msgs::Point &joint_point, const geometry_msgs::Quaternion &joint_orientation)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = arm_base_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "arm_joint_cuboids";
    marker.id = joint_idx;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(marker_lifetime_);

    // 1. 关节坐标系下的偏移转换到基坐标系
    std::vector<double> offset = cuboid_offsets_[joint_idx];
    Eigen::Quaterniond joint_quat(
        joint_orientation.w,
        joint_orientation.x,
        joint_orientation.y,
        joint_orientation.z);
    Eigen::Vector3d offset_joint(offset[0], offset[1], offset[2]);
    Eigen::Vector3d offset_base = joint_quat * offset_joint; // 关节坐标系→基坐标系
    marker.pose.position.x = joint_point.x + offset_base.x();
    marker.pose.position.y = joint_point.y + offset_base.y();
    marker.pose.position.z = joint_point.z + offset_base.z();

    // 2. 关节坐标系下的旋转增量叠加
    std::vector<double> rotate = cuboid_rotates_[joint_idx];
    double roll_inc = rotate[0], pitch_inc = rotate[1], yaw_inc = rotate[2];

    // 生成关节坐标系下的旋转增量四元数
    Eigen::Quaterniond rotate_inc = getRotateQuaternion(
        roll_inc, pitch_inc, yaw_inc,
        default_params_.default_cuboid_rotate_order);

    // 关节原始姿态 + 关节坐标系下的旋转增量
    Eigen::Quaterniond final_quat = joint_quat * rotate_inc;

    // 转换为ROS四元数
    marker.pose.orientation.x = final_quat.x();
    marker.pose.orientation.y = final_quat.y();
    marker.pose.orientation.z = final_quat.z();
    marker.pose.orientation.w = final_quat.w();

    // 尺寸和颜色
    std::vector<double> size = cuboid_sizes_[joint_idx];
    marker.scale.x = size[0];
    marker.scale.y = size[1];
    marker.scale.z = size[2];

    std::vector<double> color = cuboid_colors_[joint_idx];
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];

    return marker;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_joint_link_visualizer");
  ros::NodeHandle nh("~");

  try
  {
    ArmJointLinkVisualizer visualizer(nh);
    visualizer.run();
  }
  catch (std::exception &e)
  {
    ROS_ERROR_STREAM("Node failed: " << e.what());
    return 1;
  }

  return 0;
}
