#ifndef MULTI_ROBOT_PLANNER_H
#define MULTI_ROBOT_PLANNER_H

#include <atomic>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <json/json.h>
#include <queue>
#include <utility>

// 前置声明
class MultiRobotPlanner;

// 全局配置结构体
struct PlannerConfig
{
    double replan_frequency = 1.0;
    double replan_timeout = 60.0;
    double robot_speed = 0.1;
    double simulation_freq = 10.0;
    double conflict_distance = 0.5;
    double node_radius = 0.2;
    double edge_width = 0.05;
    std::string json_path = "./graph.json";
    bool global_is_simulation = true;
    std::string robot_config_path = "./robot_config.json";
    double node_switch_threshold = 0.8;
    int max_replan_attempts = 60;
};
extern PlannerConfig g_config;

// 拓扑节点结构体
struct TopoNode
{
    std::string name;
    double x = 0.0, y = 0.0;
    std::vector<std::string> neighbors;
    int cross_type = 0;
    int dir_type = 0;
    bool is_charge = false;
};

// 机器人配置结构体
struct RobotConfig
{
    int id = -1;
    bool is_simulation = true;
    std::string pose_topic;
    std::string goal_cmd_topic;
    double goal_arrive_threshold = 0.5;
    int priority = 5;
    std::string init_node;
    double node_switch_threshold = 0.0;
};

// 机器人状态枚举
enum class RobotState
{
    UNINITIALIZED,
    IDLE,
    RUNNING,
    WAITING,
    COMPLETED,
    FAILED
};

// 机器人核心类
class Robot
{
private:
    // 核心防护成员（原子变量改为指针）
    std::atomic<bool> *is_object_valid_ = nullptr;
    int id_ = -1;
    int priority_ = 5;
    RobotState state_ = RobotState::UNINITIALIZED;

    // 路径与位置相关
    std::string current_node_;
    std::string target_goal_;
    std::vector<std::string> path_;
    std::vector<std::string> passed_nodes_;
    std::vector<std::string> to_release_nodes_;
    size_t path_idx_ = 0;
    double x_ = 0.0, y_ = 0.0;

    // 外部依赖
    ros::NodeHandle *nh_ = nullptr;
    std::unordered_map<std::string, TopoNode> *graph_ = nullptr;
    MultiRobotPlanner *planner_ = nullptr;

    // 配置与状态
    RobotConfig config_;
    int replan_count_ = 0;
    ros::Time last_replan_time_;
    ros::Time wait_start_time_;
    bool is_wait_timeout_ = false;
    bool has_received_pose_ = false;
    bool is_initialized_ = false;

    // ROS通信
    ros::Subscriber robot_pose_sub_;
    ros::Publisher robot_goal_pub_;

    // 内部工具函数
    double calcDist(double x1, double y1, double x2, double y2);
    bool isIdValid() const { return id_ >= 0; }
    void releasePassedNodes();
    bool isNodeForbidden(const std::string &node);
    double getNodeSwitchThreshold();

public:
    // 构造/析构
    Robot(); // 空构造函数（用于map占位）
    Robot(int id, const RobotConfig &cfg, ros::NodeHandle &nh,
          std::unordered_map<std::string, TopoNode> &graph, MultiRobotPlanner &planner);
    ~Robot();

    // 手动实现移动语义（核心修复）
    Robot(Robot &&other) noexcept;
    Robot &operator=(Robot &&other) noexcept;

    // 禁用拷贝
    Robot(const Robot &) = delete;
    Robot &operator=(const Robot &) = delete;

    // 核心回调
    void robotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    // 对外接口
    int getId() const { return id_; }
    bool isInitialized() const { return is_initialized_; }
    bool isSimulationMode() const { return config_.is_simulation; }
    std::string getStateStr() const;
    double getX() const { return x_; }
    double getY() const { return y_; }
    const std::vector<std::string> &getPath() const { return path_; }
    RobotState getState() const { return state_; }
    bool isWaitTimeout();
    bool isObjectValid() const { return is_object_valid_ && is_object_valid_->load() && id_ >= 0; }

    // 任务控制
    bool setTask(const std::string &goal);
    void startTask();
    void pauseTask(bool is_conflict);
    void setPriority(int priority);
    std::vector<std::string> dijkstraWithDisconnect(const std::string &start, const std::string &goal);
    bool updateCurrentNodeByThreshold(double x, double y);
    bool initRobotPositionFromPose(double x, double y);

    // 仿真与状态更新
    bool step();
    void updateRealRobotState();
};

// 多机器人规划器类
class MultiRobotPlanner
{
private:
    // 核心数据
    ros::NodeHandle nh_;
    std::unordered_map<std::string, TopoNode> graph_;
    std::unordered_map<int, Robot> robots_;
    std::unordered_map<std::string, int> node_occupancy_;

    // ROS通信
    ros::Publisher topo_pub_;
    ros::Publisher robot_pub_;
    ros::Publisher task_result_pub_;
    ros::Subscriber goal_sub_;
    ros::Timer sim_timer_;

    // 线程安全锁
    mutable std::mutex robots_mutex_;
    mutable std::mutex node_occupancy_mutex_;

    // 内部初始化函数
    void loadConfig();
    bool loadGraph();
    void loadRobotConfigs();
    bool parseCommand(const std::string &cmd, int &robot_id, std::string &goal);
    void goalCommandCallback(const std_msgs::String::ConstPtr &msg);
    void publishTopology();
    void publishRobots();
    void checkConflictAndHandle();
    std_msgs::ColorRGBA getColor(int id);

public:
    // 构造函数
    MultiRobotPlanner(ros::NodeHandle &nh);

    // 仿真回调
    void simulationCallback(const ros::TimerEvent &);

    // 机器人管理
    bool addRobot(int id, const RobotConfig &cfg);
    bool removeRobot(int id);

    // 节点占用/释放（核心）
    bool occupyNode(const std::string &node, int robot_id);
    void releaseNode(const std::string &node, int robot_id);
    std::string assignUniqueInitNode(int robot_id);
    bool isRobotIdValid(int robot_id) const;

    // 对外访问
    const std::unordered_map<std::string, int> &getNodeOccupancy() const { return node_occupancy_; }
};

// 全局辅助函数
std::string findNearestNode(const std::unordered_map<std::string, TopoNode> &graph, double x, double y);
int clamp(int val, int min, int max);

#endif // MULTI_ROBOT_PLANNER_H