

#include "../include/odom/odom.h"
#include "../include/odom/global_defination.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_node");
    ros::NodeHandle nh;

    odom::Odom odom_pub(nh);

    ros::spin();
    ros::shutdown();
    return 0;
}
