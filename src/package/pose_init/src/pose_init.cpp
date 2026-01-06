/*
 * @Author: hambin
 * @Date: 2023-03-10 13:40:12
 * @LastEditors: Please set LastEditors
 * @FilePath: src/pose_init.cpp
 * @Description:
 */
#include "pose_init.h"
#include <signal.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace pose_init
{
    PoseInit::PoseInit(ros::NodeHandle &nhandle)
    {
        nh = nhandle;

        nh.getParam("pose_file", yaml_file);
        nh.getParam("save_period", save_period);
        nh.getParam("use_amcl_pose", use_amcl_pose);
        write_time = ros::Time::now();
        pub_init_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, true);
        ImportParams(yaml_file);
        if (!use_amcl_pose)
            sub_pose = nh.subscribe("robot_pose", 1, &PoseInit::PoseCallback, this);
        else
            sub_amcl_pose = nh.subscribe("amcl_pose", 1, &PoseInit::AmclPoseCallback, this);
    }

    void PoseInit::PoseCallback(const geometry_msgs::PoseConstPtr msg)
    {
        ros::Time cur_time = ros::Time::now();
        if ((cur_time - write_time).toSec() > save_period)
        {
            try
            {
                YAML::Node node;
                assert(node.IsNull());
                node["x"] = msg->position.x;
                node["y"] = msg->position.y;
                node["z"] = msg->position.z;
                node["ox"] = msg->orientation.x;
                node["oy"] = msg->orientation.y;
                node["oz"] = msg->orientation.z;
                node["ow"] = msg->orientation.w;

                std::ofstream file(yaml_file.c_str());
                file << node << std::endl;
                file.close();
                write_time = ros::Time::now();
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
            }
        }
    }

    void PoseInit::AmclPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr msg)
    {
        ros::Time cur_time = ros::Time::now();
        if ((cur_time - write_time).toSec() > save_period)
        {
            try
            {
                YAML::Node node;
                assert(node.IsNull());
                node["x"] = msg->pose.pose.position.x;
                node["y"] = msg->pose.pose.position.y;
                node["z"] = msg->pose.pose.position.z;
                node["ox"] = msg->pose.pose.orientation.x;
                node["oy"] = msg->pose.pose.orientation.y;
                node["oz"] = msg->pose.pose.orientation.z;
                node["ow"] = msg->pose.pose.orientation.w;

                std::ofstream file(yaml_file.c_str());
                file << node << std::endl;
                file.close();
                write_time = ros::Time::now();
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
            }
        }
    }

    bool PoseInit::ImportParams(std::string yaml_file)
    {
        try
        {
            YAML::Node doc;
            doc = YAML::LoadFile(yaml_file);
            geometry_msgs::PoseWithCovarianceStamped init_pose;
            init_pose.header.frame_id = "map";
            init_pose.header.stamp = ros::Time::now();
            init_pose.pose.pose.position.x = doc["x"].as<float>();
            init_pose.pose.pose.position.y = doc["y"].as<float>();
            init_pose.pose.pose.position.z = doc["z"].as<float>();
            init_pose.pose.pose.orientation.x = doc["ox"].as<float>();
            init_pose.pose.pose.orientation.y = doc["oy"].as<float>();
            init_pose.pose.pose.orientation.z = doc["oz"].as<float>();
            init_pose.pose.pose.orientation.w = doc["ow"].as<float>();

            double quat_norm = std::sqrt(std::pow(init_pose.pose.pose.orientation.x, 2) +
                                         std::pow(init_pose.pose.pose.orientation.y, 2) +
                                         std::pow(init_pose.pose.pose.orientation.z, 2) +
                                         std::pow(init_pose.pose.pose.orientation.w, 2));
            double tolerance = 1e-3;
            if (std::fabs(quat_norm - 1.0) > tolerance)
            {
                ROS_WARN("init_pose error");
            }
            else
                pub_init_pose.publish(init_pose);
            ROS_INFO("\033[1;32m----> init_pose send,x:%f,y:%f,z:%f.\033[0m",
                     init_pose.pose.pose.position.x, init_pose.pose.pose.position.y, init_pose.pose.pose.position.z);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            return false;
        }

        return true;
    }

} // namespace pose_init

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_init");
    ros::NodeHandle nh("~");
    pose_init::PoseInit pose(nh);

    constexpr size_t num_threads = 1;
    if (num_threads > 1)
    {
        ros::AsyncSpinner spinner(num_threads); // Uses multi threads
        spinner.start();
        ros::waitForShutdown();
    }
    else
    {
        ros::spin(); // Uses single thread
    }
    return 0;
}
