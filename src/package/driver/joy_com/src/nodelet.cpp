#include <ros/ros.h>
#include <../include/cloud_to_img/laser_to_image.hpp>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>


namespace cloud_to_img
{

class CloudToImageNodelet :public nodelet::Nodelet
{
public:
    CloudToImageNodelet() {}
    ~CloudToImageNodelet() {}

private:
  virtual void onInit()
    {
     node_.reset(new LaserToImage(getNodeHandle()));
    }
    boost::shared_ptr<LaserToImage> node_;
};
}

PLUGINLIB_EXPORT_CLASS(cloud_to_img::CloudToImageNodelet,nodelet::Nodelet);
