#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>

using namespace std::chrono_literals;

class RobotPosePublisher : public rclcpp::Node
{
public:
    RobotPosePublisher() : Node("robot_pose_publisher")
    {
        // 1. 声明并获取参数（保留ROS 1的默认值）
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<double>("publish_frequency", 10.0);
        this->declare_parameter<bool>("is_stamped", false);

        this->get_parameter("map_frame", map_frame_);
        this->get_parameter("base_frame", base_frame_);
        this->get_parameter("publish_frequency", publish_frequency_);
        this->get_parameter("is_stamped", is_stamped_);

        // 2. 根据is_stamped创建对应的发布者（设置QoS，匹配ROS 1的队列大小1）
        if (is_stamped_)
        {
            pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "robot_pose", rclcpp::QoS(1));
        }
        else
        {
            pub_ = this->create_publisher<geometry_msgs::msg::Pose>(
                "robot_pose", rclcpp::QoS(1));
        }

        // 3. 初始化TF2监听器
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 4. 等待TF变换可用（适配ROS 2的等待逻辑）
        bool tf_ready = false;
        int wait_count = 0;
        while (!tf_ready && rclcpp::ok())
        {
            try
            {
                tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
                tf_ready = true;
                RCLCPP_INFO(this->get_logger(), "TF transform from %s to %s is ready", 
                            map_frame_.c_str(), base_frame_.c_str());
            }
            catch (tf2::TransformException &ex)
            {
                wait_count++;
                if (wait_count > 10) 
                {
                    RCLCPP_WARN(this->get_logger(), "Waiting for TF transform: %s", ex.what());
                }
                rclcpp::sleep_for(1s);
            }
        }

        // 5. 创建定时器（替代ros::Rate，按指定频率发布位姿）
        double period_ms = 1000.0 / publish_frequency_;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(period_ms)),
            std::bind(&RobotPosePublisher::publish_pose, this));

        RCLCPP_INFO(this->get_logger(), "Robot pose publisher started with:");
        RCLCPP_INFO(this->get_logger(), "  map_frame: %s", map_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  base_frame: %s", base_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  publish_frequency: %.1f Hz", publish_frequency_);
        RCLCPP_INFO(this->get_logger(), "  is_stamped: %s", is_stamped_ ? "true" : "false");
    }

private:
    // 参数变量
    std::string map_frame_;
    std::string base_frame_;
    double publish_frequency_;
    bool is_stamped_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::variant<
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr,
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr> pub_;


    void publish_pose()
    {
        try
        {
            // 获取最新的TF变换（map → base_link）
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_->lookupTransform(
                    map_frame_, base_frame_, 
                    tf2::TimePointZero, // 取最新的变换
                    tf2::durationFromSec(0.1)); // 超时时间

            // 构建PoseStamped消息
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = map_frame_;
            pose_stamped.header.stamp = this->get_clock()->now();

            // 从TF变换赋值位姿
            pose_stamped.pose.orientation = transform.transform.rotation;
            pose_stamped.pose.position.x = transform.transform.translation.x;
            pose_stamped.pose.position.y = transform.transform.translation.y;
            pose_stamped.pose.position.z = transform.transform.translation.z;

            // 根据is_stamped发布对应类型
            if (is_stamped_)
            {
                std::get<0>(pub_)->publish(pose_stamped);
            }
            else
            {
                std::get<1>(pub_)->publish(pose_stamped.pose);
            }
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "Failed to get transform: %s", ex.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotPosePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}