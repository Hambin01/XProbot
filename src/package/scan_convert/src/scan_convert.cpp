/*
 * @Author: Hambin.Lu
 * @Description: ##
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <iostream>
#include <vector>
#include <signal.h>

namespace scan_convert
{
    class ScanConvert
    {
    public:
        ScanConvert(ros::NodeHandle &nhandle)
        {
            nh = nhandle;

            nh.getParam(ros::this_node::getName() + "/freq", freq);
            scan_sub_ = nh.subscribe("/scan", 2, &ScanConvert::scanCallback, this);
            map_points_pub_ = nh.advertise<sensor_msgs::PointCloud>("/scan_points", 2);
        }

        void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
        {
            sensor_msgs::PointCloud map_points;
            map_points.header = scan_msg->header;

            try
            {
                for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
                {
                    double range = scan_msg->ranges[i];
                    if (range != std::numeric_limits<float>::infinity())
                    {
                        double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
                        tf::Stamped<tf::Point> laser_point(tf::Point(range * cos(angle), range * sin(angle), 0.0), scan_msg->header.stamp, scan_msg->header.frame_id);
                        tf::Stamped<tf::Point> map_point;
                        listener.transformPoint("map", laser_point, map_point);
                        geometry_msgs::Point32 map_point_msg;
                        map_point_msg.x = map_point.getX();
                        map_point_msg.y = map_point.getY();
                        map_point_msg.z = map_point.getZ();

                        map_points.points.push_back(map_point_msg);
                    }
                }
                map_points.header.frame_id = "map";
                map_points_pub_.publish(map_points);
            }
            catch (tf::TransformException &ex)
            {
                // ROS_ERROR("Transform error: %s", ex.what());
            }
            ros::Rate(freq).sleep();
        }

    private:
        ros::NodeHandle nh;
        ros::Subscriber scan_sub_;
        ros::Publisher map_points_pub_;
        int freq = 2;
        tf::TransformListener listener;
    };

} // namespace scan_convert

static void Handler(int sig)
{
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_convert");
    ros::NodeHandle nh;
    signal(SIGINT, Handler);
    scan_convert::ScanConvert node(nh);

    ROS_INFO("\033[1;32m----> scan_convert node Started.\033[0m");
    ros::AsyncSpinner s(1);
    s.start();
    ros::waitForShutdown();

    return 0;
}
