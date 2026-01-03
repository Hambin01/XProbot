/******************************************************************************
 * Copyright leapting.com. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef _POLE_EXTRACT_H_
#define _POLE_EXTRACT_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <cmath>
#include <algorithm>
#include <string>
#include <iterator>
#include <sstream>
#include <vector>
#include <thread>
#include <mutex>
#include <queue>
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include <pcl/common/transforms.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

namespace pole_extract {

typedef pcl::PointXYZI PointType;
// typedef pcl::PointXYZINormal PointType;

class Pole_Extract {
  public:
    Pole_Extract(ros::NodeHandle nh);
    ~Pole_Extract();

  private:
    ros::Publisher pubPointCloudPole;
    ros::Subscriber subPointCloud;
    ros::Subscriber subBackGarage;
    ros::Publisher pubPolePos;
    ros::Subscriber subImu;

  private:
    void init_Data();
    void pointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &pointCloudMsg);
    void backGarageCallBack(const std_msgs::Header msg);
    void imuHandler(const sensor_msgs::ImuConstPtr &msg);
};

}  // namespace
#endif  // 