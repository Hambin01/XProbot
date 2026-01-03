#include <ros/ros.h>
#include <../include/laser_avoidance/laser_avoidance.hpp>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>


namespace laser_avoidance
{

class LaserAvoidanceNodelet :public nodelet::Nodelet
{
public:
    LaserAvoidanceNodelet() {}
    ~LaserAvoidanceNodelet() {}

private:
  virtual void onInit()
    {
     node_.reset(new LaserAvoidance(getNodeHandle()));
    }
    boost::shared_ptr<LaserAvoidance> node_;
};
}

PLUGINLIB_EXPORT_CLASS(laser_avoidance::LaserAvoidanceNodelet,nodelet::Nodelet);
