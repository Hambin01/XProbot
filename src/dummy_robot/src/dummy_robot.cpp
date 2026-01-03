#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <memory>

using namespace std::chrono_literals;

class DummyRobotNode : public rclcpp::Node
{
protected:
    std::string odom_frame_id_;
    std::string base_link_frame_id_;
    std::string init_pose_frame_id_;

    // 机器人状态
    double x_;
    double y_;
    double yaw_;
    float v_;
    float w_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_init_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfb_;
    std::shared_ptr<tf2_ros::Buffer> tfbuf_;
    std::shared_ptr<tf2_ros::TransformListener> tfl_;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    void cbTwist(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        v_ = msg->linear.x;
        w_ = msg->angular.z;
    }


    void cbInit(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped pose_in, pose_out;
        pose_in.header = msg->header;
        pose_in.pose = msg->pose.pose;
        
        try
        {
            geometry_msgs::msg::TransformStamped trans =
                tfbuf_->lookupTransform(odom_frame_id_, pose_in.header.frame_id, tf2::TimePointZero);
            tf2::doTransform(pose_in, pose_out, trans);
        }
        catch (tf2::TransformException &e)
        {
            RCLCPP_WARN(this->get_logger(), "%s", e.what());
            return;
        }

        x_ = pose_out.pose.position.x;
        y_ = pose_out.pose.position.y;
        yaw_ = tf2::getYaw(pose_out.pose.orientation);
        v_ = 0;
        w_ = 0;
    }

    void timerCallback()
    {
        const float dt = 0.01;
        const rclcpp::Time current_time = this->get_clock()->now();
        
        // 随机噪声（可选）
        // double randomNum = (static_cast<double>(rand()) / RAND_MAX) * 0.002 - 0.001;

        // 更新机器人位姿
        yaw_ += w_ * dt; // + randomNum;
        x_ += cosf(yaw_) * v_ * dt;
        y_ += sinf(yaw_) * v_ * dt;

        // 发布TF变换
        geometry_msgs::msg::TransformStamped trans;
        trans.header.stamp = current_time;
        trans.header.frame_id = odom_frame_id_;
        trans.child_frame_id = base_link_frame_id_;
        trans.transform.translation.x = x_;
        trans.transform.translation.y = y_;
        trans.transform.translation.z = 0.0;
        trans.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw_));
        tfb_->sendTransform(trans);

        // 发布里程计消息
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = odom_frame_id_;
        odom.header.stamp = current_time;
        odom.child_frame_id = base_link_frame_id_;
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw_));
        odom.twist.twist.linear.x = v_;
        odom.twist.twist.angular.z = w_;
        pub_odom_->publish(odom);
    }

public:
    DummyRobotNode() : Node("dummy_robot")
    {
        // 初始化机器人状态
        v_ = 0.0;
        w_ = 0.0;
        x_ = 0.0;
        y_ = 0.0;
        yaw_ = 0.0;

        this->declare_parameter<std::string>("odom_frame_id", "odom");
        this->declare_parameter<std::string>("base_link_frame_id", "base_link");
        this->declare_parameter<std::string>("init_pose_frame_id", "map");
        
        this->get_parameter("odom_frame_id", odom_frame_id_);
        this->get_parameter("base_link_frame_id", base_link_frame_id_);
        this->get_parameter("init_pose_frame_id", init_pose_frame_id_);

        // 初始化TF组件
        tfb_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        tfbuf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfl_ = std::make_shared<tf2_ros::TransformListener>(*tfbuf_);

        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&DummyRobotNode::cbTwist, this, std::placeholders::_1));
        sub_init_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10, std::bind(&DummyRobotNode::cbInit, this, std::placeholders::_1));

        // 创建定时器（100Hz，对应dt=0.01s）
        timer_ = this->create_wall_timer(
            10ms, std::bind(&DummyRobotNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Dummy robot node started with:");
        RCLCPP_INFO(this->get_logger(), "  odom_frame_id: %s", odom_frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "  base_link_frame_id: %s", base_link_frame_id_.c_str());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DummyRobotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}