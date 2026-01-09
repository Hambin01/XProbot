
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class DummyRobotNode
{
protected:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    double x_;
    double y_;
    double yaw_;
    float v_;
    float w_;
    std::string frame_id;
    std::string child_frame_id;

    ros::Publisher pub_odom_;
    ros::Subscriber sub_twist_;
    ros::Subscriber sub_init_;
    tf2_ros::Buffer tfbuf_;
    tf2_ros::TransformBroadcaster tfb_;
    tf2_ros::TransformListener tfl_;

    void cbTwist(const geometry_msgs::Twist::ConstPtr &msg)
    {
        v_ = msg->linear.x;
        w_ = msg->angular.z;
    }

    void cbInit(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
    {
        geometry_msgs::PoseStamped pose_in, pose_out;
        pose_in.header = msg->header;
        pose_in.pose = msg->pose.pose;
        try
        {
            geometry_msgs::TransformStamped trans =
                tfbuf_.lookupTransform(frame_id, pose_in.header.frame_id, pose_in.header.stamp, ros::Duration(1.0));
            tf2::doTransform(pose_in, pose_out, trans);
        }
        catch (tf2::TransformException &e)
        {
            ROS_WARN("%s", e.what());
            return;
        }

        x_ = pose_out.pose.position.x;
        y_ = pose_out.pose.position.y;
        yaw_ = tf2::getYaw(pose_out.pose.orientation);
        v_ = 0;
        w_ = 0;
    }

public:
    DummyRobotNode()
        : nh_(), pnh_("~"), tfl_(tfbuf_)
    {
        pnh_.param("initial_x", x_, 0.0);
        pnh_.param("initial_y", y_, 0.0);
        pnh_.param("initial_yaw", yaw_, 0.0);
        pnh_.param<std::string>("frame_id", frame_id, "odom");
        pnh_.param<std::string>("child_frame_id", child_frame_id, "base_link");
        v_ = 0.0;
        w_ = 0.0;

        pub_odom_ = nh_.advertise<nav_msgs::Odometry>("odom", 1, true);
        sub_twist_ = nh_.subscribe("cmd_vel", 1, &DummyRobotNode::cbTwist, this);
        sub_init_ = nh_.subscribe("initialpose", 1, &DummyRobotNode::cbInit, this);
    }
    void spin()
    {
        const float dt = 0.01;
        ros::Rate rate(1.0 / dt);

        while (ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
            const ros::Time current_time = ros::Time::now();
            srand(time(NULL));
            double randomNum = (static_cast<double>(rand()) / RAND_MAX) * 0.002 - 0.001;

            yaw_ += w_ * dt; //+ randomNum;
            x_ += cosf(yaw_) * v_ * dt;
            y_ += sinf(yaw_) * v_ * dt;

            geometry_msgs::TransformStamped trans;
            trans.header.stamp = current_time;
            trans.header.frame_id = frame_id;
            trans.child_frame_id = child_frame_id;
            trans.transform.translation = tf2::toMsg(tf2::Vector3(x_, y_, 0.0));
            trans.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw_));
            tfb_.sendTransform(trans);

            nav_msgs::Odometry odom;
            odom.header.frame_id = frame_id;
            odom.header.stamp = current_time;
            odom.child_frame_id = child_frame_id;
            odom.pose.pose.position.x = x_;
            odom.pose.pose.position.y = y_;
            odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw_));
            odom.twist.twist.linear.x = v_;
            odom.twist.twist.angular.z = w_;
            pub_odom_.publish(odom);
        }
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dummy_robot");

    DummyRobotNode robot;
    robot.spin();

    return 0;
}