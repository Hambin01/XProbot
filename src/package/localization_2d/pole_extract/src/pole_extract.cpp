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
#include "pole_extract.h"

namespace pole_extract {

pcl::PointCloud<PointType>::Ptr cloudPoints(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cloudPoints_pole(new pcl::PointCloud<PointType>());
int pi_x, pi_y;
double fov_x, fov_y, threshold_dis, threshold_gap, threshold_hw, resolution_x, resolution_y;
double pi_start_x, pi_start_y, rang_max, rang_min;
bool in_garage = false;
Eigen::Quaterniond q_trans_inv(1.0, 0, 0, 0);
Eigen::Quaterniond q_trans_r(1.0, 0, 0, 0);
Eigen::Vector4f trans_r(0.0, 0.0, 0.0, 1.0);
double lidar_height = 1.0;

struct deep_point {
  PointType p0;
  double deep_dis = 0.0;
  int p0_id = -1;
  int x_cluster = -1;
  int y_cluster = -1;
  int seg_start = -1;
  int seg_end = -1;
  bool flag_1 = false;
  bool flag_2 = false;
};

inline double rad_degree(double rad_in) {
  return rad_in * 180.0 / M_PI;
}
inline double degree_rad(double degree_in) {
  return degree_in * M_PI / 180.0;
}

Pole_Extract::Pole_Extract(ros::NodeHandle nh) {
  std::string cloude_input_topic, cloude_output_topic;
  ros::NodeHandle nh_param("~");
  nh_param.param<double>("rang_max", rang_max, 100.0);
  nh_param.param<double>("rang_min", rang_min, 1.0);
  nh_param.param<std::string>("cloude_input_topic", cloude_input_topic, "/rslidar_points");
  nh_param.param<std::string>("cloude_output_topic", cloude_output_topic, "/pole_points");
  nh_param.param<int>("pi_x", pi_x, 1800);
  nh_param.param<int>("pi_y", pi_y, 60);
  nh_param.param<double>("fov_x", fov_x, 360.0);
  nh_param.param<double>("fov_y", fov_y, 30.0);
  nh_param.param<double>("threshold_dis", threshold_dis, 0.3);
  nh_param.param<double>("threshold_gap", threshold_gap, 0.3);
  nh_param.param<double>("threshold_hw", threshold_hw, 2.5);
  nh_param.param<double>("lidar_height", lidar_height, 1.0);

  init_Data();

  subPointCloud = nh.subscribe<sensor_msgs::PointCloud2>(cloude_input_topic.c_str(), 10, &Pole_Extract::pointCloudHandler, this);
  pubPointCloudPole = nh.advertise<sensor_msgs::PointCloud2>(cloude_output_topic.c_str(), 10);
  subBackGarage = nh.subscribe<std_msgs::Header>("/in_garage", 10, &Pole_Extract::backGarageCallBack, this);
  // pubPolePos = nh.advertise<geometry_msgs::PoseStamped>("/gravity_pose", 10);
  subImu =  nh.subscribe<sensor_msgs::Imu>("/imu/data", 10, &Pole_Extract::imuHandler, this);
}
Pole_Extract::~Pole_Extract() {}

void Pole_Extract::backGarageCallBack(const std_msgs::Header msg) {
  if (msg.seq > 0) {
    in_garage = true;
  } else {
    in_garage = false;
  }
}

double point_dis(PointType p1, PointType p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

void points_box(pcl::PointCloud<PointType> &input_cloud, double &x_gap, double &y_gap, double &z_gap) {
  if (input_cloud.points.size() < 1) {
    ROS_WARN("input_cloud size 0; error!");
    return;
  }
  double x_min = 99999999.0, x_max = -99999999.0;
  double y_min = 99999999.0, y_max = -99999999.0;
  double z_min = 99999999.0, z_max = -99999999.0;
  for (auto pi : input_cloud.points) {
    if (pi.x < x_min) {
      x_min = pi.x;
    }
    if (pi.x > x_max) {
      x_max = pi.x;
    }
    if (pi.y < y_min) {
      y_min = pi.y;
    }
    if (pi.y > y_max) {
      y_max = pi.y;
    }
    if (pi.z < z_min) {
      z_min = pi.z;
    }
    if (pi.z > z_max) {
      z_max = pi.z;
    }
  }
  x_gap = x_max - x_min;
  y_gap = y_max - y_min;
  z_gap = z_max - z_min;
}

Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& q) {
  double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
  double roll = std::atan2(sinr_cosp, cosr_cosp);

  double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
  double pitch;
  if (std::abs(sinp) >= 1)
    pitch = std::copysign(M_PI / 2, sinp); // ????????????
  else
    pitch = std::asin(sinp);

  double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  return Eigen::Vector3d(roll, pitch, yaw);
}

Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw) {
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
  return q;
}

Eigen::Quaterniond keepOnlyYaw(const Eigen::Quaterniond& q) {
  Eigen::Vector3d euler = quaternionToEuler(q);
  double yaw = euler.z();
  return eulerToQuaternion(0.0, 0.0, yaw);
}

void Pole_Extract::imuHandler(const sensor_msgs::ImuConstPtr &msg) {

  Eigen::Quaterniond imu_q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  // Eigen::Vector3d eulerAngle = quaternionToEuler(imu_q);
  // std::cout << "roll " << eulerAngle[0] * 180.0 / M_PI << std::endl;
  // std::cout << "pitch " << eulerAngle[1] * 180.0 / M_PI << std::endl;
  // std::cout << "yaw " << eulerAngle[2] * 180.0 / M_PI << std::endl;

  // Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()));
  // Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()));
  // Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle[0], Eigen::Vector3d::UnitZ()));
  // Eigen::Quaterniond q_trans_r_m = keepOnlyYaw(imu_q);
  Eigen::Vector3d euler = quaternionToEuler(imu_q);
  double yaw = euler.z();
  Eigen::Quaterniond q_trans_r_m = eulerToQuaternion(0.0, 0.0, yaw);

  // q_trans_r_m = yawAngle * pitchAngle * rollAngle;
  // Eigen::Vector3d eulerAngle2 = q_trans_r_m.matrix().eulerAngles(2, 1, 0);
  // std::cout << "yaw2 " << eulerAngle2[0] * 180.0 / M_PI << std::endl;
  // std::cout << "pitch2 " << eulerAngle2[1] * 180.0 / M_PI << std::endl;
  // std::cout << "roll2 " << eulerAngle2[2] * 180.0 / M_PI << std::endl;
  q_trans_r = q_trans_r_m * imu_q.inverse();

  trans_r.x() = -lidar_height * sin(euler.y());
  trans_r.y() = -lidar_height * sin(euler.x());

  // // q_trans_inv = q_trans_r.inverse();
  // Eigen::Matrix3d matrix_imu = imu_q.matrix();
  // float yaw = atan2(matrix_imu(1, 0), matrix_imu(0, 0));
  // // std::cout << "yaw_j " << yaw * 180 / 3.141592653 << std::endl;
  // Eigen::Matrix3d matrix_new;
  // matrix_new << cos(yaw), -sin(yaw), 0, sin(yaw),  cos(yaw), 0, 0, 0, 1;
  // // Eigen::Matrix3d R_imu_new = matrix_imu.inverse() * matrix_new;
  // Eigen::Matrix3d R_imu_new = matrix_new * matrix_imu.inverse();
  // q_trans_r = R_imu_new.inverse();
}

void Pole_Extract::pointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &pointCloudMsg) {
  pcl::PointCloud<PointType>::Ptr rec_cloud(new pcl::PointCloud<PointType>());
  pcl::fromROSMsg(*pointCloudMsg, *rec_cloud);

  deep_point deep_arr[pi_y][pi_x];
  if (rec_cloud->points.size() == 0) {
    ROS_WARN("Pointscloud size 0; input cloud error!");
    return;
  }
  //gravity
  Eigen::Matrix4f result_T = Eigen::Matrix4f::Identity();
  Eigen::Matrix3f re_matrix = Eigen::Matrix3f::Identity();
  // re_matrix = q_trans_inv.matrix().cast<float>();
  re_matrix = q_trans_r.matrix().cast<float>();
  result_T.block<3, 3>(0, 0) = re_matrix;
  result_T.block<4, 1>(0, 3) = trans_r;
  pcl::transformPointCloud(*rec_cloud, *cloudPoints, result_T);

  // std::cout << "cloudPoints->points.size() " << cloudPoints->points.size() << std::endl;
  for (size_t i = 0; i < cloudPoints->points.size(); i++) {
    if (pcl::isFinite(cloudPoints->points[i])) {
      double p_dis = sqrt(cloudPoints->points[i].x * cloudPoints->points[i].x + cloudPoints->points[i].y * cloudPoints->points[i].y + cloudPoints->points[i].z * cloudPoints->points[i].z);
      if (p_dis <= rang_max && p_dis >= rang_min) {
        double high_z = cloudPoints->points[i].z / sqrt(cloudPoints->points[i].x * cloudPoints->points[i].x + cloudPoints->points[i].y * cloudPoints->points[i].y);
        double ang_x = atan(cloudPoints->points[i].y / cloudPoints->points[i].x);
        if (cloudPoints->points[i].x < 0 && cloudPoints->points[i].y < 0) {
          ang_x = ang_x - M_PI;
        }
        if (cloudPoints->points[i].x < 0 && cloudPoints->points[i].y > 0) {
          ang_x = M_PI + ang_x;
        }
        // std::cout << "ang_x " << rad_degree(ang_x) << std::endl;
        double rad_len_x = (ang_x + M_PI) * 1.0;
        int at_pi_x = static_cast<int>(round((rad_len_x - pi_start_x) / resolution_x));
        int at_pi_y = static_cast<int>(round((pi_start_y - high_z) / resolution_y));
        // std::cout << "at_pi_x0 " << at_pi_x << std::endl;
        // std::cout << "at_pi_y0 " << at_pi_y << std::endl;
        if (at_pi_x < 0) {
          at_pi_x = 0;
        }
        if (at_pi_y < 0) {
          at_pi_y = 0;
        }
        if (at_pi_x > pi_x - 1) {
          at_pi_x = pi_x - 1;
        }
        if (at_pi_y > pi_y - 1) {
          at_pi_y = pi_y - 1;
        }
        // std::cout << "at_pi_x1 " << at_pi_x << std::endl;
        // std::cout << "at_pi_y1 " << at_pi_y << std::endl;
        // std::cout << "ang_x " << rad_degree(ang_x) << std::endl;
        // std::cout << "(p_dis / 100.0*255.0) " << (p_dis / 100.0 * 255.0) << std::endl;
        deep_arr[at_pi_y][at_pi_x].p0 = cloudPoints->points[i];
        deep_arr[at_pi_y][at_pi_x].deep_dis = p_dis;
        deep_arr[at_pi_y][at_pi_x].p0_id = i + 1;
        deep_arr[at_pi_y][at_pi_x].x_cluster = -1;
        deep_arr[at_pi_y][at_pi_x].y_cluster = -1;
        deep_arr[at_pi_y][at_pi_x].seg_start = -1;
        deep_arr[at_pi_y][at_pi_x].seg_end = -1;
      }
    }
  }

  int cluster_couter = 0;
  for (int i = 0; i < pi_y; ++i) {//hang
    int start_x = -1;
    int hole_x = 0;
    for (int j = 0; j < pi_x; ++j) {
      if (start_x == -1) {
        if (deep_arr[i][j].p0_id > 0) {
          start_x = i * pi_x + j;
          deep_arr[i][j].x_cluster = start_x;
          deep_arr[i][j].seg_start = j;
          deep_arr[i][j].seg_end = j;
        }
      } else {
        if (hole_x < pi_x) {
          if (deep_arr[i][j].p0_id > 0) {
            if ((point_dis(deep_arr[i][j].p0, deep_arr[i][j - 1 - hole_x].p0) < threshold_gap)
                && (fabs(deep_arr[i][j].deep_dis - deep_arr[i][j - 1 - hole_x].deep_dis) < threshold_dis)) {
              deep_arr[i][j].x_cluster = deep_arr[i][j - 1 - hole_x].x_cluster;
              for (int n = 0; n < pi_x; n++) {
                if (deep_arr[i][n].x_cluster == deep_arr[i][j].x_cluster) {
                  deep_arr[i][n].seg_start = deep_arr[i][j - 1 - hole_x].seg_start;
                  deep_arr[i][n].seg_end = j;
                }
              }
              hole_x = 0;
            } else {
              start_x = i * pi_x + j;
              deep_arr[i][j].x_cluster = start_x;
              deep_arr[i][j].seg_start = j;
              deep_arr[i][j].seg_end = j;
              hole_x = 0;
            }
          } else {
            hole_x++;
          }
        } else {//5 hole
          if (deep_arr[i][j].p0_id > 0) {
            start_x = i * pi_x + j;
            deep_arr[i][j].x_cluster = start_x;
            deep_arr[i][j].seg_start = j;
            deep_arr[i][j].seg_end = j;
            hole_x = 0;
          } else {
            hole_x++;
          }
        }
      }
    }
  }
  //check cluster
  cluster_couter = -1;
  for (int k = 0; k < pi_y; k++) {
    for (int m = 0; m < pi_x; m++) {
      if (deep_arr[k][m].x_cluster > -1 && cluster_couter != deep_arr[k][m].x_cluster) {
        cluster_couter = deep_arr[k][m].x_cluster;
        if (((point_dis(deep_arr[k][deep_arr[k][m].seg_start].p0, deep_arr[k][deep_arr[k][m].seg_end].p0) < threshold_dis)
             && (fabs(deep_arr[k][deep_arr[k][m].seg_start].deep_dis - deep_arr[k][deep_arr[k][m].seg_end].deep_dis) < threshold_dis))) {
          for (int n = 0; n < pi_x; n++) {
            if (deep_arr[k][n].x_cluster == cluster_couter) {
              deep_arr[k][n].flag_1 = true;
            }
          }
        } else {
          for (int n = 0; n < pi_x; n++) {
            if (deep_arr[k][n].x_cluster == cluster_couter) {
              deep_arr[k][n].x_cluster = -1;
              deep_arr[k][n].flag_1 = false;
            }
          }
        }
      }
    }
  }


  pcl::PointCloud<PointType>::Ptr cloudPoints_row(new pcl::PointCloud<PointType>());
  for (int j = 0; j < pi_x; ++j) {
    cloudPoints_row->points.clear();
    for (int i = 0; i < pi_y; ++i) {//hang
      if (deep_arr[i][j].flag_1 == true) {
        cloudPoints_row->points.clear();
        if (deep_arr[i][j].y_cluster < 0) {
          int y_cluster_couter = j * pi_y + i;
          int s_cluster = deep_arr[i][j].x_cluster;
          int x_union_cluster = s_cluster;
          for (int m = 0; m < pi_x; m++) {
            if (s_cluster == deep_arr[i][m].x_cluster) {
              cloudPoints_row->points.push_back(deep_arr[i][m].p0);
            }
          }
          int x_union_counter = 0;
          if (i + 1 < pi_y) {
            for (int k = i + 1; k < pi_y; ++k) { //hang
              if (deep_arr[k][j].flag_1 == true) {
                if (deep_arr[k][j].y_cluster < 0) {
                  int c_cluster = deep_arr[k][j].x_cluster;
                  pcl::PointCloud<PointType>::Ptr cloudPoints_chip(new pcl::PointCloud<PointType>());
                  pcl::PointCloud<PointType>::Ptr cloudPoints_all(new pcl::PointCloud<PointType>());
                  for (int m = 0; m < pi_x; m++) {
                    if (c_cluster == deep_arr[k][m].x_cluster) {
                      cloudPoints_chip->points.push_back(deep_arr[k][m].p0);
                    }
                  }
                  *cloudPoints_all = *cloudPoints_row + *cloudPoints_chip;
                  double x_gap = -1.0, y_gap = -1.0, z_gap = -1.0;
                  points_box(*cloudPoints_all, x_gap, y_gap, z_gap);
                  if (x_gap != -1.0 && y_gap != -1.0 && x_gap < threshold_dis && y_gap < threshold_dis) {
                    x_union_counter++;
                    for (int m = 0; m < pi_x; m++) {
                      if (c_cluster == deep_arr[k][m].x_cluster) {
                        deep_arr[k][m].y_cluster = y_cluster_couter;
                        deep_arr[k][m].x_cluster = x_union_cluster;
                        deep_arr[k][m].flag_2 = true;
                      }
                    }
                  }
                }
              }
            }
          }
          if (x_union_counter > 0) {
            for (int m = 0; m < pi_x; m++) {
              if (x_union_cluster == deep_arr[i][m].x_cluster) {
                deep_arr[i][m].y_cluster = y_cluster_couter;
                deep_arr[i][m].flag_2 = true;
              }
            }
          }
        }
      }
    }
  }
  //check
  cloudPoints_row->points.clear();
  pcl::PointCloud<PointType>::Ptr cloudPoints_dir(new pcl::PointCloud<PointType>());
  int y_cluster_couter = -1;
  int x_union_cluster = -1;
  int step_counter = 0;
  for (int j = 0; j < pi_x; ++j) {
    for (int i = 0; i < pi_y; ++i) {
      if (deep_arr[i][j].flag_2 == true) {
        if (deep_arr[i][j].y_cluster > 0 && y_cluster_couter != deep_arr[i][j].y_cluster) {
          y_cluster_couter = deep_arr[i][j].y_cluster;
          x_union_cluster = deep_arr[i][j].x_cluster;
          cloudPoints_row->points.clear();
          step_counter = 0;
          for (int k = 0; k < pi_y; ++k) {//hang
            for (int m = 0; m < pi_x; ++m) {
              if (deep_arr[k][m].y_cluster == y_cluster_couter && deep_arr[k][m].x_cluster == x_union_cluster) {
                step_counter++;
                cloudPoints_row->points.push_back(deep_arr[k][m].p0);
              }
            }
          }
          double x_gap = -1.0, y_gap = -1.0, z_gap = -1.0;
          points_box(*cloudPoints_row, x_gap, y_gap, z_gap);
          if (step_counter > 2 && x_gap != 0.0 && y_gap != 0.0 && x_gap != -1.0 && y_gap != -1.0 && (fabs(z_gap / y_gap) > threshold_hw) && (fabs(z_gap / x_gap) > threshold_hw)) {
            //do nothing
          } else {
            for (int k = 0; k < pi_y; ++k) {//hang
              for (int m = 0; m < pi_x; ++m) {
                if (deep_arr[k][m].y_cluster == y_cluster_couter && deep_arr[k][m].x_cluster == x_union_cluster) {
                  deep_arr[k][m].y_cluster = 0;
                  deep_arr[k][m].flag_2 = false;
                }
              }
            }
          }
        }
      } else {
        deep_arr[i][j].y_cluster = 0;
      }
    }
  }

  cloudPoints_pole->points.clear();
  for (int i = 0; i < pi_y; ++i) {//hang
    for (int j = 0; j < pi_x; ++j) {
      if (deep_arr[i][j].y_cluster > 0) {
        cloudPoints_pole->points.push_back(deep_arr[i][j].p0);
      }
    }
  }

  if (in_garage == true) {
    cloudPoints_pole->points.clear();
    for (size_t i = 0; i < cloudPoints->points.size(); i++) {
      if (cloudPoints->points[i].x < 0.0 && cloudPoints->points[i].z < 0.3 && cloudPoints->points[i].z > 0.0) {
        cloudPoints_pole->points.push_back(cloudPoints->points[i]);
      }
    }
  }

  // static tf::TransformBroadcaster m_pubBroadcaster;
  // tf::Transform m_pole_link;
  // tf::Quaternion q_tf;
  // q_tf.setW(q_trans_inv.w());
  // q_tf.setX(q_trans_inv.x());
  // q_tf.setY(q_trans_inv.y());
  // q_tf.setZ(q_trans_inv.z());
  // m_pole_link.setRotation(q_tf);
  // m_pole_link.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  // m_pubBroadcaster.sendTransform(tf::StampedTransform(m_pole_link, pointCloudMsg->header.stamp, pointCloudMsg->header.frame_id, "pole_link"));

  // geometry_msgs::PoseStamped pole_p_m;
  // pole_p_m.header = pointCloudMsg->header;
  // pole_p_m.pose.position.x = 0.0;
  // pole_p_m.pose.position.y = 0.0;
  // pole_p_m.pose.position.z = 0.0;
  // pole_p_m.pose.orientation.x = q_trans_inv.x();
  // pole_p_m.pose.orientation.y = q_trans_inv.y();
  // pole_p_m.pose.orientation.z = q_trans_inv.z();
  // pole_p_m.pose.orientation.w = q_trans_inv.w();
  // pubPolePos.publish(pole_p_m);

  // Eigen::Matrix4f result_T = Eigen::Matrix4f::Identity();
  // Eigen::Matrix3f re_matrix = Eigen::Matrix3f::Identity();
  // re_matrix = q_trans_inv.matrix().cast<float>();

  // result_T.block<3, 3>(0, 0) = re_matrix;
  // pcl::PointCloud<PointType>::Ptr rec_cloud(new pcl::PointCloud<PointType>());
  // pcl::transformPointCloud(*cloudPoints_pole, *rec_cloud, result_T);

  sensor_msgs::PointCloud2 laserCloudPoint;
  pcl::toROSMsg(*cloudPoints_pole, laserCloudPoint);
  laserCloudPoint.header = pointCloudMsg->header;
  pubPointCloudPole.publish(laserCloudPoint);
}

void Pole_Extract::init_Data() {
  resolution_x = degree_rad(fov_x) * 1.0 / pi_x; //m/pi, 2*pi*r, r=1.0m
  resolution_y = degree_rad(fov_y) * 1.0 / pi_y; //m/pi
  pi_start_x = 0.0;
  pi_start_y = degree_rad(fov_y) * 1.0 / 2.0;
}


}



