
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseArray.h>
#include <std_srvs/Empty.h>
#include "amcl_updater/amcl_updaterConfig.h"

class amcl_updater
{
public:
    amcl_updater();

    void paramCallback(amcl_updater_cfg::amcl_updaterConfig& config, uint32_t level);

private:
    void init_param();

    void call_timer(const ros::TimerEvent& e);
    void spin_call();
    void particlecloud_sub(const geometry_msgs::PoseArray::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;
    ros::ServiceClient call_client_;
    ros::Subscriber particlecloud_sub_;
    ros::Timer call_timer_;

    double call_frequency_;
    int min_particles_;
    bool init_flag_;
    double time_flag_;
    double time_tolerance_;

    dynamic_reconfigure::Server<amcl_updater_cfg::amcl_updaterConfig> server;
    dynamic_reconfigure::Server<amcl_updater_cfg::amcl_updaterConfig>::CallbackType f;
};

amcl_updater::amcl_updater()
:ph_("~")
{
    init_flag_ = false;
    init_param();

    call_client_ = nh_.serviceClient<std_srvs::Empty>("request_nomotion_update");
    particlecloud_sub_ = nh_.subscribe<geometry_msgs::PoseArray>("particlecloud", 1000, &amcl_updater::particlecloud_sub, this);
    call_timer_ = nh_.createTimer(1.0, &amcl_updater::call_timer, this);

    f = boost::bind(&amcl_updater::paramCallback, this, _1, _2);
    server.setCallback(f);

    sleep(20);
    spin_call();
    init_flag_ = true;
    time_flag_ = ros::Time::now().toSec();
    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //     spin_call();
    //     usleep(1e6 / call_frequency_);
    // }
}

void amcl_updater::init_param()
{
    ph_.param("call_frequency", call_frequency_, 5.0);
    ph_.param("min_particles", min_particles_, 0);
    ph_.param("time_tolerance", time_tolerance_, 2.0);
}

void amcl_updater::paramCallback(amcl_updater_cfg::amcl_updaterConfig& config, uint32_t level)
{
    call_frequency_ = config.call_frequency;
    min_particles_ = config.min_particles;
}

void amcl_updater::spin_call()
{
    std_srvs::Empty srv;
    if (!call_client_.call(srv))
    {
        ROS_WARN("failed to call service");
    }
}

void amcl_updater::particlecloud_sub(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    geometry_msgs::PoseArray particles;
    if (min_particles_ > 0)
    {
        for (int i = 0; i < msg->poses.size(); i++)
        {
            bool new_particles = true;
            for (int j = 0; j < particles.poses.size(); j++)
            {
                if ((fabs(msg->poses[i].position.x - particles.poses[j].position.x) < 1e-6)
                && (fabs(msg->poses[i].position.y - particles.poses[j].position.y) < 1e-6)
                && (fabs(msg->poses[i].position.z - particles.poses[j].position.z) < 1e-6)
                && (fabs(msg->poses[i].orientation.w - particles.poses[j].orientation.w) < 1e-6)
                && (fabs(msg->poses[i].orientation.x - particles.poses[j].orientation.x) < 1e-6)
                && (fabs(msg->poses[i].orientation.y - particles.poses[j].orientation.y) < 1e-6)
                && (fabs(msg->poses[i].orientation.z - particles.poses[j].orientation.z) < 1e-6))
                {
                    new_particles = false;
                    break;
                }
            }
            if (new_particles)
            {
                particles.poses.push_back(msg->poses[i]);
            }
        }
        ROS_WARN("amcl_updater particles pose's size: %d", (int)particles.poses.size());
    }
    if ((min_particles_ <= 0) || (particles.poses.size() > min_particles_))
    {
        if (call_frequency_ > 0)
        {
            usleep(1e6 / call_frequency_);
            ros::spinOnce();
            spin_call();
            time_flag_ = ros::Time::now().toSec();
        }
    }
}

void amcl_updater::call_timer(const ros::TimerEvent& e)
{
    if (init_flag_ && call_frequency_ > 0)
    {
        if (ros::Time::now().toSec() - time_flag_ > time_tolerance_)
        {
            spin_call();
            time_flag_ = ros::Time::now().toSec();
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "amcl_updater");
    amcl_updater amcl_updater;

    ros::spin();
}
