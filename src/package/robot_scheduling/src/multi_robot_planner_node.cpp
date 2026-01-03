#include "multi_robot_planner.h"

PlannerConfig g_config;

// 辅助函数：数值钳位
int clamp(int val, int min, int max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

// 辅助函数：查找最近节点
std::string findNearestNode(const std::unordered_map<std::string, TopoNode> &graph, double x, double y)
{
    if (graph.empty())
        return "";

    std::string nearest_node;
    double min_dist = INFINITY;

    for (const auto &pair : graph)
    {
        const TopoNode &node = pair.second;
        double dist = sqrt(pow(x - node.x, 2) + pow(y - node.y, 2));
        if (dist < min_dist)
        {
            min_dist = dist;
            nearest_node = pair.first;
        }
    }

    return nearest_node;
}

// ------------------------------
// Robot类实现（核心修复）
// ------------------------------
Robot::Robot()
    : id_(-1), nh_(nullptr), graph_(nullptr), planner_(nullptr),
      has_received_pose_(false), is_initialized_(false), state_(RobotState::UNINITIALIZED),
      is_object_valid_(new std::atomic<bool>(false)) {}

Robot::Robot(int id, const RobotConfig &cfg, ros::NodeHandle &nh,
             std::unordered_map<std::string, TopoNode> &graph, MultiRobotPlanner &planner)
    : id_(-1), config_(cfg), nh_(&nh), graph_(&graph), planner_(&planner),
      has_received_pose_(false), is_initialized_(false), state_(RobotState::UNINITIALIZED),
      is_object_valid_(new std::atomic<bool>(false))
{
    if (id < 0)
    {
        ROS_ERROR("Failed to create Robot - ID is negative (ID=%d)", id);
        return;
    }

    id_ = id;
    priority_ = clamp(cfg.priority, 0, 9);
    is_object_valid_->store(true);

    ROS_INFO("[Robot %d] Constructor initialized - is_simulation: %s, priority: %d",
             id_, config_.is_simulation ? "true" : "false", priority_);

    if (config_.is_simulation)
    {
        current_node_ = cfg.init_node.empty() ? planner.assignUniqueInitNode(id) : cfg.init_node;
        if (current_node_.empty() || !graph.count(current_node_))
        {
            current_node_ = graph.begin()->first;
            ROS_WARN("[Robot %d] Invalid init node, use default: %s", id_, current_node_.c_str());
        }

        x_ = graph[current_node_].x;
        y_ = graph[current_node_].y;
        planner_->occupyNode(current_node_, id_);
        is_initialized_ = true;
        state_ = RobotState::IDLE;

        ROS_INFO("[Robot %d] Simulation mode initialized - current node: %s, coordinates(%.2f, %.2f)",
                 id_, current_node_.c_str(), x_, y_);
    }
    else
    {
        std::string pose_topic = cfg.pose_topic.empty() ? "/robot_" + std::to_string(id_) + "/pose" : cfg.pose_topic;
        std::string goal_topic = cfg.goal_cmd_topic.empty() ? "/robot_" + std::to_string(id_) + "/goal_cmd" : cfg.goal_cmd_topic;

        robot_pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
            pose_topic, 10, &Robot::robotPoseCallback, this);
        robot_goal_pub_ = nh.advertise<std_msgs::String>(goal_topic, 10);

        ROS_INFO("[Robot %d] Real mode initialized - subscribe pose: %s, publish goal: %s",
                 id_, pose_topic.c_str(), goal_topic.c_str());
    }
}

Robot::~Robot()
{
    if (is_object_valid_)
    {
        is_object_valid_->store(false);
        delete is_object_valid_;
        is_object_valid_ = nullptr;
    }
    if (nh_)
    {
        robot_pose_sub_.shutdown();
        robot_goal_pub_.shutdown();
    }
    // 仅当ID有效时打印析构日志（核心修复：消除-1 ID日志）
    if (id_ >= 0)
    {
        ROS_INFO("[Robot %d] Object destroyed - marked as invalid", id_);
    }
}

// 手动实现移动构造（核心修复：不重置源对象ID）
Robot::Robot(Robot &&other) noexcept
{
    // 转移原子变量指针，并为源对象创建新的无效标记
    is_object_valid_ = other.is_object_valid_;
    other.is_object_valid_ = new std::atomic<bool>(false);

    // 转移所有成员（不重置源对象ID）
    id_ = other.id_;
    priority_ = other.priority_;
    state_ = other.state_;
    current_node_ = std::move(other.current_node_);
    target_goal_ = std::move(other.target_goal_);
    path_ = std::move(other.path_);
    passed_nodes_ = std::move(other.passed_nodes_);
    to_release_nodes_ = std::move(other.to_release_nodes_);
    path_idx_ = other.path_idx_;
    x_ = other.x_;
    y_ = other.y_;

    // 转移指针成员
    nh_ = other.nh_;
    other.nh_ = nullptr;
    graph_ = other.graph_;
    other.graph_ = nullptr;
    planner_ = other.planner_;
    other.planner_ = nullptr;

    // 转移配置与状态
    config_ = std::move(other.config_);
    replan_count_ = other.replan_count_;
    last_replan_time_ = other.last_replan_time_;
    wait_start_time_ = other.wait_start_time_;
    is_wait_timeout_ = other.is_wait_timeout_;
    has_received_pose_ = other.has_received_pose_;
    is_initialized_ = other.is_initialized_;

    // 转移ROS通信对象
    robot_pose_sub_ = std::move(other.robot_pose_sub_);
    robot_goal_pub_ = std::move(other.robot_goal_pub_);
}

// 手动实现移动赋值（核心修复：不重置源对象ID）
Robot &Robot::operator=(Robot &&other) noexcept
{
    if (this != &other)
    {
        // 释放当前原子变量
        if (is_object_valid_)
        {
            delete is_object_valid_;
        }

        // 转移原子变量指针，并为源对象创建新的无效标记
        is_object_valid_ = other.is_object_valid_;
        other.is_object_valid_ = new std::atomic<bool>(false);

        // 转移所有成员（不重置源对象ID）
        id_ = other.id_;
        priority_ = other.priority_;
        state_ = other.state_;
        current_node_ = std::move(other.current_node_);
        target_goal_ = std::move(other.target_goal_);
        path_ = std::move(other.path_);
        passed_nodes_ = std::move(other.passed_nodes_);
        to_release_nodes_ = std::move(other.to_release_nodes_);
        path_idx_ = other.path_idx_;
        x_ = other.x_;
        y_ = other.y_;

        // 转移指针成员
        nh_ = other.nh_;
        other.nh_ = nullptr;
        graph_ = other.graph_;
        other.graph_ = nullptr;
        planner_ = other.planner_;
        other.planner_ = nullptr;

        // 转移配置与状态
        config_ = std::move(other.config_);
        replan_count_ = other.replan_count_;
        last_replan_time_ = other.last_replan_time_;
        wait_start_time_ = other.wait_start_time_;
        is_wait_timeout_ = other.is_wait_timeout_;
        has_received_pose_ = other.has_received_pose_;
        is_initialized_ = other.is_initialized_;

        // 转移ROS通信对象
        robot_pose_sub_ = std::move(other.robot_pose_sub_);
        robot_goal_pub_ = std::move(other.robot_goal_pub_);
    }
    return *this;
}

double Robot::calcDist(double x1, double y1, double x2, double y2)
{
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

void Robot::releasePassedNodes()
{
    if (!isObjectValid() || !planner_ || to_release_nodes_.empty())
        return;

    for (const auto &node : to_release_nodes_)
    {
        if (node == target_goal_)
            continue;
        planner_->releaseNode(node, id_);
    }
    to_release_nodes_.clear();
}

bool Robot::isNodeForbidden(const std::string &node)
{
    if (!isObjectValid() || !planner_)
        return false;

    const auto &node_occupancy = planner_->getNodeOccupancy();
    auto occ_it = node_occupancy.find(node);
    if (occ_it != node_occupancy.end() && occ_it->second != id_)
    {
        return true;
    }
    return false;
}

bool Robot::isWaitTimeout()
{
    if (!isObjectValid() || state_ != RobotState::WAITING)
        return false;

    double elapsed = (ros::Time::now() - wait_start_time_).toSec();
    if (elapsed >= g_config.replan_timeout)
    {
        is_wait_timeout_ = true;
        return true;
    }
    return false;
}

double Robot::getNodeSwitchThreshold()
{
    if (config_.node_switch_threshold > 0.0)
    {
        return config_.node_switch_threshold;
    }
    return g_config.node_switch_threshold;
}

bool Robot::updateCurrentNodeByThreshold(double x, double y)
{
    if (!isObjectValid() || !graph_ || graph_->empty() || current_node_.empty())
        return false;

    double switch_threshold = getNodeSwitchThreshold();
    double dist_to_current = calcDist(x, y, (*graph_)[current_node_].x, (*graph_)[current_node_].y);

    if (dist_to_current <= switch_threshold)
    {
        return false;
    }

    std::string new_node = findNearestNode(*graph_, x, y);
    if (new_node.empty() || new_node == current_node_)
    {
        return false;
    }

    ROS_DEBUG("Robot %d: Switch node from %s to %s (dist: %.2f > threshold: %.2f)",
              id_, current_node_.c_str(), new_node.c_str(), dist_to_current, switch_threshold);

    if (current_node_ != target_goal_ &&
        std::find(passed_nodes_.begin(), passed_nodes_.end(), current_node_) == passed_nodes_.end())
    {
        passed_nodes_.push_back(current_node_);
        to_release_nodes_.push_back(current_node_);
    }

    releasePassedNodes();
    planner_->releaseNode(current_node_, id_);
    planner_->occupyNode(new_node, id_);

    current_node_ = new_node;
    x_ = x;
    y_ = y;

    return true;
}

bool Robot::initRobotPositionFromPose(double x, double y)
{
    if (!isObjectValid() || !graph_ || !planner_ || is_initialized_)
        return false;

    std::string nearest_node = findNearestNode(*graph_, x, y);
    if (nearest_node.empty())
    {
        ROS_WARN("Robot %d: No nearest node found for pose (%.2f, %.2f)", id_, x, y);
        return false;
    }

    if (!planner_->occupyNode(nearest_node, id_))
    {
        ROS_ERROR("Robot %d failed to occupy initial node '%s'", id_, nearest_node.c_str());
        return false;
    }

    x_ = x;
    y_ = y;
    current_node_ = nearest_node;
    is_initialized_ = true;
    state_ = RobotState::IDLE;

    ROS_INFO("Robot %d initialized from pose: (%.2f, %.2f) -> node %s",
             id_, x, y, nearest_node.c_str());

    return true;
}

void Robot::robotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if (!isObjectValid())
    {
        ROS_ERROR("[Robot INVALID] Pose callback failed - Object is invalid (ID=%d, valid_flag=%d)",
                  id_, is_object_valid_ ? is_object_valid_->load() : false);
        return;
    }

    ROS_DEBUG("[Robot %d] Received pose message - is_simulation: %s, has_received_pose: %s",
              id_, config_.is_simulation ? "true" : "false",
              has_received_pose_ ? "true" : "false");

    if (config_.is_simulation)
    {
        ROS_WARN("[Robot %d] Skip pose processing - Simulation mode enabled", id_);
        return;
    }

    has_received_pose_ = true;
    double new_x = msg->pose.position.x;
    double new_y = msg->pose.position.y;

    ROS_INFO("[Robot %d] Real robot pose updated - coordinates(%.2f, %.2f), current node: %s, initialized: %s",
             id_, new_x, new_y, current_node_.c_str(), is_initialized_ ? "true" : "false");

    if (!is_initialized_)
    {
        initRobotPositionFromPose(new_x, new_y);
        return;
    }

    updateCurrentNodeByThreshold(new_x, new_y);
    x_ = new_x;
    y_ = new_y;

    ROS_DEBUG("[Robot %d] Pose processing completed - final coordinates(%.2f, %.2f), current node: %s",
              id_, x_, y_, current_node_.c_str());
}

std::string Robot::getStateStr() const
{
    switch (state_)
    {
    case RobotState::UNINITIALIZED:
        return "UNINITIALIZED";
    case RobotState::IDLE:
        return "IDLE";
    case RobotState::RUNNING:
        return "RUNNING";
    case RobotState::WAITING:
        return "WAITING";
    case RobotState::COMPLETED:
        return "COMPLETED";
    case RobotState::FAILED:
        return "FAILED";
    default:
        return "UNKNOWN";
    }
}

void Robot::setPriority(int priority)
{
    if (!isObjectValid())
    {
        ROS_ERROR("Failed to set priority - Robot object is invalid (ID=%d)", id_);
        return;
    }
    if (!isIdValid())
    {
        ROS_ERROR("Failed to set priority - Robot ID is invalid (ID=%d)", id_);
        return;
    }

    priority_ = clamp(priority, 0, 9);
    config_.priority = priority_;
    ROS_INFO("[Robot %d] Priority updated to %d", id_, priority_);
}

std::vector<std::string> Robot::dijkstraWithDisconnect(const std::string &start, const std::string &goal)
{
    if (!isObjectValid())
    {
        ROS_ERROR("Failed to run Dijkstra - Robot object is invalid (ID=%d)", id_);
        return {};
    }

    if (!isIdValid() || !graph_ || !planner_ || !is_initialized_ ||
        graph_->find(start) == graph_->end() || graph_->find(goal) == graph_->end())
    {
        ROS_ERROR("Failed to run Dijkstra - Invalid parameters (ID=%d, start=%s, goal=%s, initialized=%s)",
                  id_, start.c_str(), goal.c_str(), is_initialized_ ? "true" : "false");
        return {};
    }

    if (isNodeForbidden(goal))
    {
        ROS_WARN("[Robot %d] Goal node '%s' is forbidden", id_, goal.c_str());
        return {};
    }

    std::unordered_map<std::string, double> dist;
    std::unordered_map<std::string, std::string> prev;
    using PQElement = std::pair<double, std::string>;
    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<>> pq;

    for (const auto &pair : *graph_)
    {
        dist[pair.first] = isNodeForbidden(pair.first) ? INFINITY : INFINITY;
    }
    dist[start] = 0.0;
    pq.push({0.0, start});

    while (!pq.empty())
    {
        auto [d, curr] = pq.top();
        pq.pop();

        if (curr == goal)
            break;
        if (d > dist[curr])
            continue;

        const TopoNode &curr_node = (*graph_)[curr];
        for (const std::string &neighbor : curr_node.neighbors)
        {
            if (isNodeForbidden(neighbor))
                continue;

            double cost = calcDist(curr_node.x, curr_node.y, (*graph_)[neighbor].x, (*graph_)[neighbor].y);
            double new_dist = d + cost;

            if (new_dist < dist[neighbor])
            {
                dist[neighbor] = new_dist;
                prev[neighbor] = curr;
                pq.push({new_dist, neighbor});
            }
        }
    }

    std::vector<std::string> path;
    for (std::string curr = goal; prev.find(curr) != prev.end(); curr = prev[curr])
    {
        path.push_back(curr);
    }

    if (!path.empty())
    {
        path.push_back(start);
        std::reverse(path.begin(), path.end());

        for (const auto &node : path)
        {
            if (isNodeForbidden(node))
            {
                ROS_WARN("[Robot %d] Path contains forbidden node '%s' - clear path", id_, node.c_str());
                path.clear();
                break;
            }
        }

        if (!path.empty())
        {
            for (size_t i = 1; i < path.size(); ++i)
            {
                planner_->occupyNode(path[i], id_);
            }
            ROS_INFO("[Robot %d] Dijkstra path found - start: %s, goal: %s, path length: %lu",
                     id_, start.c_str(), goal.c_str(), path.size());
        }
    }
    else
    {
        ROS_WARN("[Robot %d] No valid path found - start: %s, goal: %s", id_, start.c_str(), goal.c_str());
    }

    return path;
}

bool Robot::setTask(const std::string &goal)
{
    if (!isObjectValid())
    {
        ROS_ERROR("Cannot set task - Robot object is invalid (ID=%d)", id_);
        return false;
    }

    if (!isIdValid() || !nh_ || !graph_ || !planner_ || !is_initialized_)
    {
        ROS_WARN("Robot %d cannot set task - Not initialized", id_);
        return false;
    }

    if (state_ == RobotState::RUNNING)
    {
        ROS_WARN("Robot %d cannot set task - Already running", id_);
        return false;
    }

    releasePassedNodes();
    passed_nodes_.clear();
    to_release_nodes_.clear();

    for (size_t i = 1; i < path_.size(); ++i)
    {
        planner_->releaseNode(path_[i], id_);
    }

    path_.clear();
    path_idx_ = 0;
    target_goal_ = goal;

    path_ = dijkstraWithDisconnect(current_node_, goal);

    if (!config_.is_simulation && !path_.empty())
    {
        std_msgs::String goal_msg;
        goal_msg.data = target_goal_;
        robot_goal_pub_.publish(goal_msg);
        ROS_INFO("[Robot %d] Publish goal node '%s' to real robot", id_, target_goal_.c_str());
    }

    if (path_.empty())
    {
        state_ = RobotState::WAITING;
        replan_count_ = 0;
        is_wait_timeout_ = false;
        last_replan_time_ = ros::Time::now();
        wait_start_time_ = ros::Time::now();
        ROS_WARN("[Robot %d] Task set failed - Empty path, state changed to WAITING", id_);
    }
    else
    {
        state_ = RobotState::IDLE;
        ROS_INFO("[Robot %d] Task set successfully - goal: %s, state changed to IDLE", id_, goal.c_str());
    }

    return true;
}

void Robot::startTask()
{
    if (!isObjectValid())
    {
        ROS_ERROR("Cannot start task - Robot object is invalid (ID=%d)", id_);
        return;
    }

    if (!isIdValid() || target_goal_.empty() || !is_initialized_)
    {
        ROS_WARN("Robot %d cannot start task - Not initialized or no goal set", id_);
        return;
    }

    if (state_ == RobotState::IDLE || state_ == RobotState::WAITING)
    {
        state_ = RobotState::RUNNING;
        ROS_INFO("[Robot %d] Task started - goal: %s, state changed to RUNNING", id_, target_goal_.c_str());

        if (!config_.is_simulation)
        {
            std_msgs::String goal_msg;
            goal_msg.data = target_goal_;
            robot_goal_pub_.publish(goal_msg);
            ROS_INFO("[Robot %d] Republish goal node '%s' to real robot", id_, target_goal_.c_str());
        }
    }
    else
    {
        ROS_WARN("[Robot %d] Cannot start task - Current state: %s", id_, getStateStr().c_str());
    }
}

void Robot::pauseTask(bool is_conflict)
{
    if (!isObjectValid())
    {
        ROS_ERROR("Cannot pause task - Robot object is invalid (ID=%d)", id_);
        return;
    }

    if (!isIdValid() || state_ != RobotState::RUNNING)
    {
        ROS_WARN("Robot %d cannot pause task - Not running (state: %s)", id_, getStateStr().c_str());
        return;
    }

    state_ = is_conflict ? RobotState::WAITING : RobotState::IDLE;
    wait_start_time_ = ros::Time::now();
    replan_count_ = 0;
    ROS_INFO("[Robot %d] Task paused - conflict: %s, state changed to %s",
             id_, is_conflict ? "true" : "false", getStateStr().c_str());

    if (!config_.is_simulation)
    {
        std_msgs::String pause_msg;
        pause_msg.data = "PAUSE";
        robot_goal_pub_.publish(pause_msg);
        ROS_INFO("[Robot %d] Publish pause command to real robot", id_);
    }
}

bool Robot::step()
{
    if (!isObjectValid())
    {
        ROS_ERROR("Cannot step simulation - Robot object is invalid (ID=%d)", id_);
        return false;
    }

    if (!isIdValid())
    {
        ROS_ERROR("Cannot step simulation - Robot ID is invalid (ID=%d)", id_);
        return false;
    }

    if (!config_.is_simulation || !is_initialized_ || state_ != RobotState::RUNNING || path_.empty())
    {
        return false;
    }

    if (path_idx_ >= path_.size() - 1)
    {
        state_ = RobotState::COMPLETED;
        ROS_INFO("[Robot %d] Simulation completed - Reached goal: %s", id_, target_goal_.c_str());
        return false;
    }

    const TopoNode &curr_node = (*graph_)[path_[path_idx_]];
    const TopoNode &next_node = (*graph_)[path_[path_idx_ + 1]];

    double dist = calcDist(x_, y_, next_node.x, next_node.y);
    if (dist <= g_config.robot_speed)
    {
        passed_nodes_.push_back(curr_node.name);
        to_release_nodes_.push_back(curr_node.name);
        releasePassedNodes();

        planner_->releaseNode(curr_node.name, id_);
        planner_->occupyNode(next_node.name, id_);

        x_ = next_node.x;
        y_ = next_node.y;
        current_node_ = next_node.name;
        path_idx_++;

        ROS_DEBUG("[Robot %d] Simulation step - Moved to node %s, path index: %lu",
                  id_, next_node.name.c_str(), path_idx_);
    }
    else
    {
        double ratio = g_config.robot_speed / dist;
        x_ += (next_node.x - x_) * ratio;
        y_ += (next_node.y - y_) * ratio;

        ROS_DEBUG("[Robot %d] Simulation step - Moved to (%.2f, %.2f), remaining distance to next node: %.2f",
                  id_, x_, y_, dist - g_config.robot_speed);
    }

    return true;
}

void Robot::updateRealRobotState()
{
    if (!isObjectValid())
    {
        ROS_ERROR("Cannot update real robot state - Object is invalid (ID=%d)", id_);
        return;
    }

    if (!isIdValid())
    {
        ROS_ERROR("Cannot update real robot state - ID is invalid (ID=%d)", id_);
        return;
    }

    if (config_.is_simulation || !is_initialized_)
    {
        return;
    }

    if (state_ != RobotState::RUNNING || target_goal_.empty())
    {
        return;
    }

    double dist_to_goal = calcDist(x_, y_, (*graph_)[target_goal_].x, (*graph_)[target_goal_].y);
    if (dist_to_goal <= config_.goal_arrive_threshold)
    {
        state_ = RobotState::COMPLETED;
        ROS_INFO("[Robot %d] Real robot completed - Reached goal: %s (distance: %.2f <= threshold: %.2f)",
                 id_, target_goal_.c_str(), dist_to_goal, config_.goal_arrive_threshold);

        std_msgs::String complete_msg;
        complete_msg.data = "COMPLETED:" + target_goal_;
        robot_goal_pub_.publish(complete_msg);
    }
}

// ------------------------------
// MultiRobotPlanner类实现（核心修复addRobot）
// ------------------------------
void MultiRobotPlanner::loadConfig()
{
    nh_.param("replan_frequency", g_config.replan_frequency, g_config.replan_frequency);
    nh_.param("replan_timeout", g_config.replan_timeout, g_config.replan_timeout);
    nh_.param("robot_speed", g_config.robot_speed, g_config.robot_speed);
    nh_.param("simulation_freq", g_config.simulation_freq, g_config.simulation_freq);
    nh_.param("conflict_distance", g_config.conflict_distance, g_config.conflict_distance);
    nh_.param("node_radius", g_config.node_radius, g_config.node_radius);
    nh_.param("edge_width", g_config.edge_width, g_config.edge_width);
    nh_.param("json_path", g_config.json_path, g_config.json_path);
    nh_.param("global_is_simulation", g_config.global_is_simulation, g_config.global_is_simulation);
    nh_.param("robot_config_path", g_config.robot_config_path, g_config.robot_config_path);
    nh_.param("node_switch_threshold", g_config.node_switch_threshold, g_config.node_switch_threshold);
    g_config.max_replan_attempts = static_cast<int>(g_config.replan_timeout * g_config.replan_frequency);

    ROS_INFO("Planner config loaded - json_path: %s, robot_config_path: %s, global_simulation: %s",
             g_config.json_path.c_str(), g_config.robot_config_path.c_str(),
             g_config.global_is_simulation ? "true" : "false");
}

bool MultiRobotPlanner::loadGraph()
{
    std::ifstream graph_file(g_config.json_path);
    if (!graph_file.is_open())
    {
        ROS_ERROR("Failed to open graph file: %s", g_config.json_path.c_str());
        return false;
    }

    Json::Value root;
    Json::CharReaderBuilder reader_builder;
    std::string errs;
    if (!Json::parseFromStream(reader_builder, graph_file, &root, &errs))
    {
        ROS_ERROR("Failed to parse graph file: %s, error: %s", g_config.json_path.c_str(), errs.c_str());
        return false;
    }

    for (const Json::Value &node_val : root)
    {
        TopoNode node;
        node.name = node_val["name"].asString();
        node.x = node_val["pose"]["position"]["x"].asDouble();
        node.y = node_val["pose"]["position"]["y"].asDouble();
        node.cross_type = node_val["cross_type"].asInt();
        node.dir_type = node_val["dir_type"].asInt();
        node.is_charge = node_val["is_charge"].asBool();

        const Json::Value &neighbors = node_val["connect_node"];
        for (const Json::Value &n : neighbors)
        {
            node.neighbors.push_back(n.asString());
        }

        graph_[node.name] = node;
        ROS_DEBUG("Loaded topology node - name: %s, x: %.2f, y: %.2f, neighbors: %lu",
                  node.name.c_str(), node.x, node.y, node.neighbors.size());
    }

    ROS_INFO("Topology graph loaded - total nodes: %lu", graph_.size());
    return !graph_.empty();
}

void MultiRobotPlanner::loadRobotConfigs()
{
    std::ifstream robot_file(g_config.robot_config_path);
    if (!robot_file.is_open())
    {
        ROS_WARN("Failed to open robot config file: %s", g_config.robot_config_path.c_str());
        return;
    }

    Json::Value root;
    Json::CharReaderBuilder reader_builder;
    std::string errs;
    if (!Json::parseFromStream(reader_builder, robot_file, &root, &errs))
    {
        ROS_ERROR("Failed to parse robot config file: %s, error: %s", g_config.robot_config_path.c_str(), errs.c_str());
        return;
    }

    const Json::Value &robots = root["robots"];
    for (const Json::Value &r : robots)
    {
        RobotConfig cfg;
        cfg.id = r["id"].asInt();
        if (cfg.id < 0)
        {
            ROS_WARN("Skip invalid robot config - ID is negative (ID=%d)", cfg.id);
            continue;
        }

        cfg.is_simulation = r["is_simulation"].asBool();
        cfg.pose_topic = r["pose_topic"].asString();
        cfg.goal_cmd_topic = r["goal_cmd_topic"].asString();
        cfg.goal_arrive_threshold = r["goal_arrive_threshold"].asDouble();
        cfg.priority = r["priority"].asInt();
        cfg.init_node = r["init_node"].asString();
        cfg.node_switch_threshold = r["node_switch_threshold"].asDouble();

        addRobot(cfg.id, cfg);
        ROS_INFO("Loaded robot config - ID: %d, simulation: %s, init_node: %s",
                 cfg.id, cfg.is_simulation ? "true" : "false", cfg.init_node.c_str());
    }

    ROS_INFO("Robot configs loaded - total robots: %lu", robots_.size());
}

std_msgs::ColorRGBA MultiRobotPlanner::getColor(int id)
{
    std_msgs::ColorRGBA color;
    switch (id % 8)
    {
    case 0:
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        break;
    case 1:
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        break;
    case 2:
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        break;
    case 3:
        color.r = 1.0;
        color.g = 1.0;
        color.b = 0.0;
        break;
    case 4:
        color.r = 1.0;
        color.g = 0.0;
        color.b = 1.0;
        break;
    case 5:
        color.r = 0.0;
        color.g = 1.0;
        color.b = 1.0;
        break;
    case 6:
        color.r = 0.5;
        color.g = 0.5;
        color.b = 0.5;
        break;
    case 7:
        color.r = 1.0;
        color.g = 0.5;
        color.b = 0.0;
        break;
    default:
        color.r = 1.0;
        color.g = 1.0;
        color.b = 1.0;
        break;
    }
    color.a = 1.0;
    return color;
}

void MultiRobotPlanner::publishTopology()
{
    visualization_msgs::MarkerArray markers;
    int marker_id = 0;

    // Draw nodes
    for (const auto &pair : graph_)
    {
        const TopoNode &node = pair.second;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "nodes";
        marker.id = marker_id++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = node.x;
        marker.pose.position.y = node.y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = g_config.node_radius * 2;
        marker.scale.y = g_config.node_radius * 2;
        marker.scale.z = 0.1;
        marker.color = getColor(marker_id);
        marker.lifetime = ros::Duration(0.1);

        markers.markers.push_back(marker);

        // Draw node name
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "node_names";
        text_marker.id = marker_id++;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position.x = node.x;
        text_marker.pose.position.y = node.y + 0.2;
        text_marker.pose.position.z = 0.0;
        text_marker.pose.orientation.w = 1.0;
        text_marker.scale.z = 0.2;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        text_marker.text = node.name;
        text_marker.lifetime = ros::Duration(0.1);

        markers.markers.push_back(text_marker);
    }

    // Draw edges
    for (const auto &pair : graph_)
    {
        const TopoNode &node = pair.second;
        for (const std::string &neighbor : node.neighbors)
        {
            if (graph_.find(neighbor) == graph_.end())
                continue;
            const TopoNode &neigh_node = graph_[neighbor];

            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "edges";
            marker.id = marker_id++;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = g_config.edge_width;
            marker.color = getColor(marker_id);
            marker.lifetime = ros::Duration(0.1);

            geometry_msgs::Point p1, p2;
            p1.x = node.x;
            p1.y = node.y;
            p1.z = 0.0;
            p2.x = neigh_node.x;
            p2.y = neigh_node.y;
            p2.z = 0.0;

            marker.points.push_back(p1);
            marker.points.push_back(p2);

            markers.markers.push_back(marker);
        }
    }

    topo_pub_.publish(markers);
}

void MultiRobotPlanner::publishRobots()
{
    visualization_msgs::MarkerArray markers;
    int marker_id = 0;

    std::lock_guard<std::mutex> lock(robots_mutex_);
    for (const auto &pair : robots_)
    {
        const Robot &robot = pair.second;
        if (!robot.isObjectValid())
            continue;

        // Draw robot position
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "robots";
        marker.id = marker_id++;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = robot.getX();
        marker.pose.position.y = robot.getY();
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = g_config.node_radius * 1.5;
        marker.scale.y = g_config.node_radius * 1.5;
        marker.scale.z = 0.2;
        marker.color = getColor(robot.getId());
        marker.lifetime = ros::Duration(0.1);

        markers.markers.push_back(marker);

        // Draw robot ID
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "robot_ids";
        text_marker.id = marker_id++;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position.x = robot.getX();
        text_marker.pose.position.y = robot.getY() + 0.3;
        text_marker.pose.position.z = 0.0;
        text_marker.pose.orientation.w = 1.0;
        text_marker.scale.z = 0.3;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        text_marker.text = "R" + std::to_string(robot.getId()) + " (" + robot.getStateStr() + ")";
        text_marker.lifetime = ros::Duration(0.1);

        markers.markers.push_back(text_marker);

        // Draw robot path
        const std::vector<std::string> &path = robot.getPath();
        if (!path.empty())
        {
            visualization_msgs::Marker path_marker;
            path_marker.header.frame_id = "map";
            path_marker.header.stamp = ros::Time::now();
            path_marker.ns = "robot_paths";
            path_marker.id = marker_id++;
            path_marker.type = visualization_msgs::Marker::LINE_STRIP;
            path_marker.action = visualization_msgs::Marker::ADD;
            path_marker.pose.orientation.w = 1.0;
            path_marker.scale.x = g_config.edge_width * 2;
            path_marker.color = getColor(robot.getId());
            path_marker.color.a = 0.7;
            path_marker.lifetime = ros::Duration(0.1);

            for (const std::string &node_name : path)
            {
                if (graph_.find(node_name) == graph_.end())
                    continue;
                geometry_msgs::Point p;
                p.x = graph_[node_name].x;
                p.y = graph_[node_name].y;
                p.z = 0.0;
                path_marker.points.push_back(p);
            }

            markers.markers.push_back(path_marker);
        }
    }

    robot_pub_.publish(markers);
}

void MultiRobotPlanner::checkConflictAndHandle()
{
    std::lock_guard<std::mutex> lock(robots_mutex_);
    std::vector<std::pair<int, int>> conflicting_pairs;

    // Check distance between robots
    for (auto it1 = robots_.begin(); it1 != robots_.end(); ++it1)
    {
        Robot &r1 = it1->second;
        if (!r1.isObjectValid() || !r1.isInitialized() || r1.getState() != RobotState::RUNNING)
            continue;

        for (auto it2 = std::next(it1); it2 != robots_.end(); ++it2)
        {
            Robot &r2 = it2->second;
            if (!r2.isObjectValid() || !r2.isInitialized() || r2.getState() != RobotState::RUNNING)
                continue;

            double dist = sqrt(pow(r1.getX() - r2.getX(), 2) + pow(r1.getY() - r2.getY(), 2));
            if (dist < g_config.conflict_distance)
            {
                conflicting_pairs.emplace_back(r1.getId(), r2.getId());
                ROS_WARN("Conflict detected - Robot %d and %d (distance: %.2f < %.2f)",
                         r1.getId(), r2.getId(), dist, g_config.conflict_distance);
            }
        }
    }

    // Handle conflicts (pause lower priority robot)
    for (const auto &pair : conflicting_pairs)
    {
        int id1 = pair.first;
        int id2 = pair.second;

        Robot &r1 = robots_[id1];
        Robot &r2 = robots_[id2];

        if (r1.isObjectValid() && r2.isObjectValid())
        {
            if (r1.getState() == RobotState::RUNNING && r2.getState() == RobotState::RUNNING)
            {
                if (r1.getId() > r2.getId())
                { // Simple priority: smaller ID has higher priority
                    r1.pauseTask(true);
                    ROS_INFO("Handle conflict - Pause Robot %d (lower priority than %d)", id1, id2);
                }
                else
                {
                    r2.pauseTask(true);
                    ROS_INFO("Handle conflict - Pause Robot %d (lower priority than %d)", id2, id1);
                }
            }
        }
    }
}

bool MultiRobotPlanner::parseCommand(const std::string &cmd, int &robot_id, std::string &goal)
{
    size_t sep = cmd.find(':');
    if (sep == std::string::npos)
    {
        ROS_WARN("Invalid command format - %s (expected: <robot_id>:<goal_node>)", cmd.c_str());
        return false;
    }

    try
    {
        robot_id = std::stoi(cmd.substr(0, sep));
        goal = cmd.substr(sep + 1);
    }
    catch (...)
    {
        ROS_WARN("Failed to parse command - %s", cmd.c_str());
        return false;
    }

    if (!isRobotIdValid(robot_id))
    {
        ROS_WARN("Invalid robot ID in command - %d", robot_id);
        return false;
    }

    if (graph_.find(goal) == graph_.end())
    {
        ROS_WARN("Invalid goal node in command - %s", goal.c_str());
        return false;
    }

    return true;
}

void MultiRobotPlanner::goalCommandCallback(const std_msgs::String::ConstPtr &msg)
{
    int robot_id;
    std::string goal;

    if (parseCommand(msg->data, robot_id, goal))
    {
        std::lock_guard<std::mutex> lock(robots_mutex_);
        Robot &robot = robots_[robot_id];
        if (robot.setTask(goal))
        {
            robot.startTask();
            ROS_INFO("Execute goal command - Robot %d: move to %s", robot_id, goal.c_str());
        }
        else
        {
            ROS_WARN("Failed to execute goal command - Robot %d: move to %s", robot_id, goal.c_str());
        }
    }
    else
    {
        ROS_WARN("Invalid goal command - %s", msg->data.c_str());
    }
}

MultiRobotPlanner::MultiRobotPlanner(ros::NodeHandle &nh) : nh_(nh)
{
    loadConfig();
    if (!loadGraph())
    {
        ROS_FATAL("Failed to load topology graph - exit");
        ros::shutdown();
        return;
    }

    loadRobotConfigs();

    // Initialize ROS publishers/subscribers
    topo_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/topology", 10);
    robot_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/robots", 10);
    task_result_pub_ = nh.advertise<std_msgs::String>("/task_results", 10);
    goal_sub_ = nh.subscribe<std_msgs::String>("/goal_commands", 10, &MultiRobotPlanner::goalCommandCallback, this);

    // Start simulation timer
    sim_timer_ = nh.createTimer(ros::Duration(1.0 / g_config.simulation_freq), &MultiRobotPlanner::simulationCallback, this);

    ROS_INFO("MultiRobotPlanner initialized successfully");
}

void MultiRobotPlanner::simulationCallback(const ros::TimerEvent &)
{
    std::lock_guard<std::mutex> lock(robots_mutex_);

    checkConflictAndHandle();

    for (auto &pair : robots_)
    {
        Robot &robot = pair.second;
        if (!robot.isObjectValid())
        {
            ROS_WARN("Skip invalid Robot %d in simulation", robot.getId());
            continue;
        }

        if (robot.isSimulationMode() && robot.isInitialized())
        {
            robot.step();
        }
        else if (!robot.isSimulationMode() && robot.isInitialized())
        {
            robot.updateRealRobotState();
        }
    }

    publishTopology();
    publishRobots();
}

// 核心修复：直接emplace构造，避免临时对象
bool MultiRobotPlanner::addRobot(int id, const RobotConfig &cfg)
{
    std::lock_guard<std::mutex> lock(robots_mutex_);

    if (id < 0)
    {
        ROS_ERROR("Failed to add Robot - ID is negative (ID=%d)", id);
        return false;
    }

    if (robots_.find(id) != robots_.end())
    {
        ROS_WARN("Robot %d already exists - skip", id);
        return false;
    }

    // 直接在map中构造对象，彻底消除临时对象
    auto [it, success] = robots_.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(id),
        std::forward_as_tuple(id, cfg, nh_, graph_, *this));

    if (success)
    {
        ROS_INFO("Added Robot %d to planner - total robots: %lu", id, robots_.size());
        return true;
    }
    else
    {
        ROS_ERROR("Failed to emplace Robot %d into planner", id);
        return false;
    }
}

bool MultiRobotPlanner::removeRobot(int id)
{
    std::lock_guard<std::mutex> lock(robots_mutex_);

    auto it = robots_.find(id);
    if (it == robots_.end())
    {
        ROS_WARN("Failed to remove Robot %d - not found", id);
        return false;
    }

    robots_.erase(it);
    ROS_INFO("Removed Robot %d from planner - total robots: %lu", id, robots_.size());
    return true;
}

bool MultiRobotPlanner::occupyNode(const std::string &node, int robot_id)
{
    if (!isRobotIdValid(robot_id))
    {
        ROS_ERROR("Failed to occupy node '%s' - Robot ID %d does not exist", node.c_str(), robot_id);

        std::lock_guard<std::mutex> lock(robots_mutex_);
        ROS_ERROR("Current valid robot IDs:");
        for (const auto &pair : robots_)
        {
            ROS_ERROR("  - ID: %d", pair.first);
        }
        return false;
    }

    std::lock_guard<std::mutex> lock(node_occupancy_mutex_);
    auto it = node_occupancy_.find(node);
    if (it != node_occupancy_.end() && it->second != robot_id)
    {
        ROS_WARN("Failed to occupy node '%s' - occupied by Robot %d", node.c_str(), it->second);
        return false;
    }

    node_occupancy_[node] = robot_id;
    ROS_DEBUG("Robot %d occupied node '%s'", robot_id, node.c_str());
    return true;
}

void MultiRobotPlanner::releaseNode(const std::string &node, int robot_id)
{
    if (!isRobotIdValid(robot_id))
    {
        ROS_ERROR("Failed to release node '%s' - Robot ID %d does not exist", node.c_str(), robot_id);
        return;
    }

    std::lock_guard<std::mutex> lock(node_occupancy_mutex_);
    auto it = node_occupancy_.find(node);
    if (it != node_occupancy_.end() && it->second == robot_id)
    {
        node_occupancy_.erase(it);
        ROS_DEBUG("Robot %d released node '%s'", robot_id, node.c_str());
    }
    else if (it != node_occupancy_.end())
    {
        ROS_WARN("Failed to release node '%s' - occupied by Robot %d (not %d)",
                 node.c_str(), it->second, robot_id);
    }
    else
    {
        ROS_WARN("Failed to release node '%s' - not occupied", node.c_str());
    }
}

std::string MultiRobotPlanner::assignUniqueInitNode(int robot_id)
{
    std::vector<std::string> free_nodes;
    for (const auto &pair : graph_)
    {
        const TopoNode &node = pair.second;
        if (!node.is_charge && node_occupancy_.find(node.name) == node_occupancy_.end())
        {
            free_nodes.push_back(node.name);
        }
    }

    if (free_nodes.empty())
    {
        ROS_WARN("No free nodes available - assign first node");
        return graph_.begin()->first;
    }

    int idx = (robot_id - 1) % free_nodes.size();
    ROS_DEBUG("Assign initial node to Robot %d - %s", robot_id, free_nodes[idx].c_str());
    return free_nodes[idx];
}

bool MultiRobotPlanner::isRobotIdValid(int robot_id) const
{
    std::lock_guard<std::mutex> lock(robots_mutex_);
    return robot_id >= 0 && robots_.find(robot_id) != robots_.end();
}

// ------------------------------
// Main function
// ------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_robot_planner_node");
    ros::NodeHandle nh("~");

    try
    {
        MultiRobotPlanner planner(nh);
        ros::spin();
    }
    catch (const std::exception &e)
    {
        ROS_FATAL("Planner exception: %s", e.what());
        return 1;
    }

    return 0;
}