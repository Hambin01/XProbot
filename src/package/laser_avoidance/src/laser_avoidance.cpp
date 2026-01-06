#include "../include/laser_avoidance/laser_avoidance.hpp"

namespace laser_avoidance
{

  LaserAvoidance::LaserAvoidance(ros::NodeHandle nh, ros::NodeHandle pnh)
  {
    nhandle = nh;
    phandle = pnh;

    phandle.param<bool>("adjust_debug", adjust_debug, false);
    phandle.param<int>("max_fitter", max_fitter, 100);
    phandle.param<double>("offset", offset, 0.0);
    phandle.param<double>("min_range", min_range, 0.1);
    phandle.param<double>("max_range", max_range, 10.0);
    pub_laser_filter = nhandle.advertise<sensor_msgs::LaserScan>("/laser_filter", 2);
    pub_adjust_req = nhandle.advertise<std_msgs::Int8>("/adjust_req", 2);
    pub_adjust_motion = nhandle.advertise<robot_state_msgs::data_to_stm32>("/data_to_stm32", 2);
    pub_adjust_motion_cmd_vel = nhandle.advertise<geometry_msgs::Twist>("/cmd_vel", 2);

    sub_laser_msgs = nhandle.subscribe("/scan", 5, &LaserAvoidance::LaserHandler, this);
    sub_adjust_cmd = nhandle.subscribe("/adjust_cmd", 5, &LaserAvoidance::AdjustSetHandler, this);
    sub_debug_cmd = nhandle.subscribe("/debug_cmd", 5, &LaserAvoidance::DebugSetHandler, this);
    std::string file_name;
    phandle.param<std::string>("file_name", file_name, "");
  }

  LaserAvoidance::~LaserAvoidance()
  {
  }

  void LaserAvoidance::AdjustSetHandler(const std_msgs::Int8ConstPtr &msg)
  {
    adjust_flag = msg->data;
  }

  void LaserAvoidance::DebugSetHandler(const std_msgs::Int8ConstPtr &msg)
  {
    adjust_debug = msg->data;
  }

  void LaserAvoidance::LaserHandler(const sensor_msgs::LaserScanConstPtr &scan)
  {
    if (adjust_debug == 1 || adjust_flag == 1)
      LaserFilter(*scan);
  }

  void LaserAvoidance::LaserFilter(const sensor_msgs::LaserScan &msg)
  {
    sensor_msgs::PointCloud2 cloud;
    sensor_msgs::LaserScan laser_msg = msg;
    int count = 0;
    for (int i = 0; i < laser_msg.ranges.size(); i++)
    {
      if (laser_msg.intensities[i] > max_fitter && (laser_msg.ranges[i] > min_range && laser_msg.ranges[i] < max_range))
      {
        count++;
      }
      else
      {
        // std::cout<< laser_msg.intensities[i]<<std::endl;
        laser_msg.ranges[i] = 0;
      }
    }
    // std::cout<< count<<std::endl;
    pub_laser_filter.publish(laser_msg);

    if (adjust_flag != 1 && adjust_debug != 1)
      return;

    if (!tfListener.waitForTransform(laser_msg.header.frame_id, "base_link",
                                     laser_msg.header.stamp + ros::Duration().fromSec(laser_msg.ranges.size() * laser_msg.time_increment), ros::Duration(1)))
    {
      ROS_WARN("no tranfroms to base_link!");
      return;
    }
    projector.transformLaserScanToPointCloud("base_link", laser_msg, cloud, tfListener);

    float x_total = 0;
    float y_total = 0;
    count = 0;
    uintptr_t cp = (uintptr_t)cloud.data.data();
    for (uint32_t j = 0; j < cloud.height; ++j)
      for (uint32_t i = 0; i < cloud.width; ++i)
      {
        float *fp = (float *)(cp + (i + j * cloud.width) * cloud.point_step);
        double x = fp[0];
        double y = fp[1];
        //************calculate label*********************//
        if (fabs(x) < 0.2 && fabs(y) > 0.1)
        {
          x_total += x;
          y_total += y;
          count++;
        }
      }
    if (count > 2)
    {
      x_total = x_total / count;
      y_total = y_total / count;
      x_total = x_total - offset;

      if (adjust_debug)
      {
        if (fabs(x_total) <= 0.003)
        {
          ROS_INFO("yes,it's okay");
        }
        else
        {
          ROS_INFO("adjust x total: %f,please move x in 0.003", x_total);
        }
        return;
      }

      robot_state_msgs::data_to_stm32 cmd;
      geometry_msgs::Twist cmd_vel;
      cmd.task_type = 2;
      cmd.running.type = 1;
      if (fabs(x_total) > 0.003)
      {
        cmd.running.speed = (x_total) > 0 ? 0.005 : -0.005;
        cmd_vel.linear.x = cmd.running.speed;
      }
      else
      {
        cmd.running.speed = 0;
        cmd_vel.linear.x = 0;
        adjust_flag = false;
        std_msgs::Int8 req;
        req.data = 1;
        pub_adjust_req.publish(req);
        ROS_INFO("adjust ok!");
      }
      pub_adjust_motion.publish(cmd);
      pub_adjust_motion_cmd_vel.publish(cmd_vel);
    }
    else
    {
      adjust_flag = false;
      std_msgs::Int8 req;
      req.data = 1;
      pub_adjust_req.publish(req);
      ROS_INFO("no data received, adjust failed!");
    }
  }

}
