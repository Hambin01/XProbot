#include "robot_control.h"
#include <QMessageBox>
#include <QFileDialog>
#include <fstream>
#include <iostream>
#include <pluginlib/class_list_macros.hpp>
#include <stdexcept>
#include <cmath>
#include <rclcpp/logging.hpp>

namespace robot_control
{

// 构造函数：仅初始化父类
RobotControl::RobotControl(QWidget *parent)
    : rviz_common::Panel(parent)
{
}

// 析构函数：清理资源
RobotControl::~RobotControl()
{
    if (moveTimer) {
        moveTimer->cancel();
        moveTimer.reset();
    }
    pubKeepTask.reset();
    pubOneTask.reset();
    pubRobotCmd.reset();
    pubGoalSend.reset();
    pubNavigationStart.reset();
    pubNodeList.reset();
    pubGoalList.reset();
    
    subRobotPose.reset();
    subRobotState.reset();
    subClickPoint.reset();
}

// 核心初始化：纯独立节点，无任何RViz上下文依赖（关键修复）
void RobotControl::onInitialize()
{
    // 1. 直接创建独立ROS2节点（适配所有RViz2版本，无任何头文件依赖）
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr); // 确保ROS2已初始化
    }
    nh_ = rclcpp::Node::make_shared("robot_control_panel");

    // 2. 初始化UI
    ui_.setupUi(this);

    // 3. 初始化下拉框
    ui_.crossBox->addItem("NO_LOAD");
    ui_.crossBox->addItem("UP_LOAD_RIGHT");
    ui_.crossBox->addItem("UP_LOAD_LEFT");
    ui_.crossBox->addItem("DOWN_LOAD_RIGHT");
    ui_.crossBox->addItem("DOWN_LOAD_LEFT");

    // 4. 创建发布者
    pubKeepTask = nh_->create_publisher<robot_task_msgs::msg::RobotKeepTaskSend>("/keep_task_send", 1);
    pubOneTask = nh_->create_publisher<robot_task_msgs::msg::RobotOneTaskSend>("/one_task_send", 1);
    pubRobotCmd = nh_->create_publisher<robot_state_msgs::msg::DataToStm32>("data_to_stm32", 2);
    pubGoalSend = nh_->create_publisher<robot_task_msgs::msg::RobotGoal>("/goal_node", 1);
    pubNavigationStart = nh_->create_publisher<robot_task_msgs::msg::RobotNavigationCmd>("/navigation_set", 1);
    pubNodeList = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("/node_list", 1);
    pubGoalList = nh_->create_publisher<robot_task_msgs::msg::RobotGoalList>("/goal_list", 1);

    // 5. 创建订阅者
    subRobotPose = nh_->create_subscription<nav_msgs::msg::Odometry>(
        "/robot_pose", 1,
        std::bind(&RobotControl::RobotPoseCallBack, this, std::placeholders::_1));
    subRobotState = nh_->create_subscription<robot_state_msgs::msg::RobotState>(
        "/robot_state", 1,
        std::bind(&RobotControl::StateCallBack, this, std::placeholders::_1));
    subClickPoint = nh_->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 1,
        std::bind(&RobotControl::ClickPointCallBack, this, std::placeholders::_1));

    // 6. 创建定时器
    moveTimer = nh_->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&RobotControl::CmdSet, this));

    // 7. 初始化指令默认值
    cmd.task_type = 0;
    cmd.running.speed = 0.0;
    cmd.running.radius = 0.0;
    cmd.cross.type = 0;
    cmd.putter.hight = 0;

    // 8. 初始化节点
    QString filename = "/install/nodes/my.json";
    // import_nodes(filename.toStdString());

    RCLCPP_INFO(nh_->get_logger(), "RobotControl panel initialized successfully!");
}

// RViz配置保存
void RobotControl::save(rviz_common::Config config) const
{
    rviz_common::Panel::save(config);
}

// RViz配置加载
void RobotControl::load(const rviz_common::Config &config)
{
    rviz_common::Panel::load(config);
}

// 定时器回调
void RobotControl::CmdSet()
{
    if (!nh_->get_node_base_interface() || !rclcpp::ok()) {
        RCLCPP_WARN(nh_->get_logger(), "ROS2 node invalid, skip cmd publish");
        return;
    }

    if (robotState.current_task == RobotCharging)
        robotMoveFlag = false;

    if (robotMoveFlag)
        pubRobotCmd->publish(cmd);

    if (robotChargeFlag && (robotState.current_task == RobotIdle || robotState.current_task == RobotRadiusRunning))
    {
        if (move_count < 25)
        {
            move_count++;
            pubRobotCmd->publish(cmd);
        }
        else
        {
            robotChargeFlag = false;
            move_count = 0;
            cmd.running.speed = 0;
            pubRobotCmd->publish(cmd);
        }
    }
}

// 位姿回调
void RobotControl::RobotPoseCallBack(const nav_msgs::msg::Odometry::SharedPtr pose)
{
    try {
        std::unique_lock<std::shared_mutex> lockDeal(mutexReadPose);
        robotPose = *pose;
        ui_.robot_pose_x->setValue(robotPose.pose.pose.position.x);
        ui_.robot_pose_y->setValue(robotPose.pose.pose.position.y);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(nh_->get_logger(), "Pose callback error: %s", e.what());
    }
}

// 状态回调
void RobotControl::StateCallBack(const robot_state_msgs::msg::RobotState::SharedPtr data)
{
    try {
        ui_.curTask->setValue(data->current_task);
        ui_.battery->setValue(data->battery_capacity > 100 ? 100 : data->battery_capacity);
        ui_.warnning->setValue(data->warning_code);
        ui_.erroCode->setValue(data->error_code);
        ui_.ultrasonic->setValue(data->ultrasonic_value);
        ui_.chargeState->setValue(data->charge_state);
        ui_.speed->setValue(data->speed);
        ui_.radius->setValue(data->radius);
        robotState = *data;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(nh_->get_logger(), "State callback error: %s", e.what());
    }
}

// 点击点回调
void RobotControl::ClickPointCallBack(const geometry_msgs::msg::PointStamped::SharedPtr point)
{
    try {
        if (connectFlag)
        {
            for (auto &it : nodeList)
            {
                if (hypot(it.second.pose.position.x - point->point.x, 
                          it.second.pose.position.y - point->point.y) < 0.1)
                {
                    std::pair<std::string, std::string> connectPoint;
                    if (line_lists.empty() || firstConnectFlag)
                    {
                        connectPoint = std::make_pair(it.first, it.first);
                        firstConnectFlag = false;
                    }
                    else
                    {
                        connectPoint = std::make_pair(line_lists.back().second, it.first);
                        nodeList[it.first].connect_node.push_back(line_lists.back().second);
                        nodeList[line_lists.back().second].connect_node.push_back(it.first);
                    }
                    line_lists.push_back(connectPoint);
                    break;
                }
            }
        }

        if (deleteFlag)
        {
            static std::string firstNode;
            for (auto &it : nodeList)
            {
                if (hypot(it.second.pose.position.x - point->point.x, 
                          it.second.pose.position.y - point->point.y) < 0.1)
                {
                    if (firstDeleteFlag)
                    {
                        firstNode = it.first;
                        firstDeleteFlag = false;
                        break;
                    }
                    else
                    {
                        disconnect_two_points(firstNode, it.first);
                        firstNode = it.first;
                        break;
                    }
                }
            }
        }

        pub_node_list();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(nh_->get_logger(), "Click point callback error: %s", e.what());
    }
}

// 断开节点连线
void RobotControl::disconnect_two_points(std::string first, std::string second)
{
    try {
        auto &conn1 = nodeList[first].connect_node;
        for (auto it = conn1.begin(); it != conn1.end(); ++it) {
            if (*it == second) {
                conn1.erase(it);
                break;
            }
        }

        auto &conn2 = nodeList[second].connect_node;
        for (auto it = conn2.begin(); it != conn2.end(); ++it) {
            if (*it == first) {
                conn2.erase(it);
                break;
            }
        }

        line_lists.clear();
        for (auto &it : nodeList) {
            for (auto &conn : it.second.connect_node) {
                line_lists.emplace_back(it.first, conn);
            }
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(nh_->get_logger(), "Disconnect points error: %s", e.what());
    }
}

// Pose转JSON
nlohmann::json RobotControl::convert_pose_to_json(geometry_msgs::msg::Pose pose)
{
    nlohmann::json node;
    node["position"]["x"] = pose.position.x;
    node["position"]["y"] = pose.position.y;
    node["position"]["z"] = pose.position.z;
    node["orientation"]["x"] = pose.orientation.x;
    node["orientation"]["y"] = pose.orientation.y;
    node["orientation"]["z"] = pose.orientation.z;
    node["orientation"]["w"] = pose.orientation.w;
    return node;
}

// JSON转Pose
geometry_msgs::msg::Pose RobotControl::json_to_pose(const nlohmann::json &list)
{
    geometry_msgs::msg::Pose pose;
    try {
        pose.position.x = list["pose"]["position"]["x"].get<float>();
        pose.position.y = list["pose"]["position"]["y"].get<float>();
        pose.position.z = list["pose"]["position"]["z"].get<float>();

        pose.orientation.x = list["pose"]["orientation"]["x"].get<float>();
        pose.orientation.y = list["pose"]["orientation"]["y"].get<float>();
        pose.orientation.z = list["pose"]["orientation"]["z"].get<float>();
        pose.orientation.w = list["pose"]["orientation"]["w"].get<float>();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(nh_->get_logger(), "JSON to pose error: %s", e.what());
        pose.position.x = pose.position.y = pose.position.z = 0.0;
        pose.orientation.w = 1.0;
    }
    return pose;
}

// 转换节点为Marker
visualization_msgs::msg::Marker RobotControl::convert_node_to_mark(int id,
                                                                   std::string name,
                                                                   geometry_msgs::msg::Pose &pose,
                                                                   int type)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = nh_->now();
    marker.ns = "basic_shapes";
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.id = id;

    if (type == 0)
    {
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.scale.z = 0.2;
        marker.color.b = 1.0;
        marker.color.g = 1.0;
        marker.color.r = 0.0;
        marker.color.a = 1.0;
        marker.text = name;
        marker.pose = pose;
    }
    else if (type == 1)
    {
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.b = 0.0;
        marker.color.g = 170.0/255.0;
        marker.color.r = 250.0/255.0;
        marker.color.a = 0.5;
        marker.pose = pose;
    }
    else if (type == 2)
    {
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.4;
        marker.color.b = 0.0;
        marker.color.g = 0.0;
        marker.color.r = 250.0/255.0;
        marker.color.a = 0.5;
        marker.pose = pose;
    }
    else if (type == 3)
    {
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.b = 0.0;
        marker.color.g = 250.0/255.0;
        marker.color.r = 0.0;
        marker.color.a = 0.5;
        marker.pose = pose;
    }

    return marker;
}

// 连接节点为连线Marker
void RobotControl::connect_two_points(visualization_msgs::msg::Marker &line_list, 
                                     geometry_msgs::msg::Pose &p1, 
                                     geometry_msgs::msg::Pose &p2)
{
    line_list.header.frame_id = "map";
    line_list.header.stamp = nh_->now();
    line_list.ns = "basic_shapes";
    line_list.action = visualization_msgs::msg::Marker::ADD;
    line_list.id = 2;
    line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
    line_list.scale.x = 0.025;
    line_list.color.b = 25.0/255.0;
    line_list.color.g = 0.0;
    line_list.color.r = 20.0/255.0;
    line_list.color.a = 0.5;

    geometry_msgs::msg::Point p;
    p.x = p1.position.x;
    p.y = p1.position.y;
    p.z = p1.position.z;
    line_list.points.push_back(p);

    p.x = p2.position.x;
    p.y = p2.position.y;
    p.z = p2.position.z;
    line_list.points.push_back(p);
}

// 发布节点列表
void RobotControl::pub_node_list(void)
{
    try {
        visualization_msgs::msg::MarkerArray markerArray;
        robot_task_msgs::msg::RobotGoalList goalList;
        markerArray.markers.clear();
        pubNodeList->publish(markerArray);

        int i = 0;
        for (auto &it : nodeList)
        {
            std::string name = it.first;
            geometry_msgs::msg::Pose pose = it.second.pose;
            markerArray.markers.push_back(convert_node_to_mark(i++, name, pose, 0));
            markerArray.markers.push_back(convert_node_to_mark(i++, name, pose, 
                it.second.is_charge ? 2 : (it.second.dir_type == 0 ? 1 : 3)));
            goalList.node.push_back(it.second);
        }

        visualization_msgs::msg::Marker lines;
        if (!line_lists.empty())
        {
            for (auto &line : line_lists)
                connect_two_points(lines, nodeList[line.first].pose, nodeList[line.second].pose);
            markerArray.markers.push_back(lines);
        }

        pubGoalList->publish(goalList);
        pubNodeList->publish(markerArray);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(nh_->get_logger(), "Publish node list error: %s", e.what());
    }
}

// ------------------------ UI槽函数 ------------------------
void RobotControl::on_robot_go_pressed()
{
    cmd.task_type = 2;
    cmd.running.type = 1;
    cmd.running.speed = ui_.robot_speed->value();
    cmd.running.radius = 0;
    robotMoveFlag = true;
}

void RobotControl::on_robot_go_released()
{
    robotMoveFlag = false;
}

void RobotControl::on_robot_back_pressed()
{
    cmd.task_type = 2;
    cmd.running.type = 1;
    cmd.running.speed = -ui_.robot_speed->value();
    cmd.running.radius = 0;
    robotMoveFlag = true;
}

void RobotControl::on_robot_back_released()
{
    robotMoveFlag = false;
}

void RobotControl::on_robot_left_pressed()
{
    cmd.task_type = 2;
    cmd.running.type = 1;
    cmd.running.speed = 0;
    cmd.running.radius = -ui_.robot_radius->value();
    robotMoveFlag = true;
}

void RobotControl::on_robot_left_released()
{
    robotMoveFlag = false;
}

void RobotControl::on_robot_right_pressed()
{
    cmd.task_type = 2;
    cmd.running.type = 1;
    cmd.running.speed = 0;
    cmd.running.radius = ui_.robot_radius->value();
    robotMoveFlag = true;
}

void RobotControl::on_robot_right_released()
{
    robotMoveFlag = false;
}

void RobotControl::on_setGoal_clicked()
{
    try {
        robot_task_msgs::msg::RobotGoal goal;
        goal.name = ui_.comboBox->currentText().toStdString();
        goal.pose = nodeList[goal.name].pose;
        goal.putter_hight = ui_.putterTaskHeight->value();
        goal.cross_type = ui_.crossBox->currentIndex();
        ui_.goal_pose_x->setValue(goal.pose.position.x);
        ui_.goal_pose_y->setValue(goal.pose.position.y);

        pubGoalSend->publish(goal);
    } catch (const std::exception &e) {
        QMessageBox::critical(this, "Error", "Set goal failed: " + QString(e.what()));
    }
}

void RobotControl::on_insertNode_clicked()
{
    if (ui_.newNodeName->text().isEmpty())
    {
        QMessageBox::warning(this, "Warning", "Node name cannot be empty!");
        return;
    }

    if (ui_.newNodeName->isModified())
    {
        std::string node_name = ui_.newNodeName->text().toStdString();
        if (nodeList.find(node_name) == nodeList.end())
        {
            ui_.comboBox->addItem(ui_.newNodeName->text());
            nodeList[node_name].pose = robotPose.pose.pose;
        }
        else
        {
            QMessageBox::StandardButton result = QMessageBox::warning(
                this, "Warning", "This node already exists! Are you sure to replace it?", 
                QMessageBox::Yes | QMessageBox::No);
            if (result == QMessageBox::Yes)
            {
                nodeList[node_name].pose = robotPose.pose.pose;
            }
        }
        pub_node_list();
    }
}

void RobotControl::on_nodeClear_clicked()
{
    ui_.comboBox->clear();
    nodeList.clear();
    line_lists.clear();
    pub_node_list();
    QMessageBox::information(this, "Info", "Node list cleared!");
}

void RobotControl::on_importNode_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(
        this, "Open File", "/install/nodes", "JSON File(*.json)");
    if (fileName.isEmpty()) return;

    try {
        import_nodes(fileName.toStdString());
        QMessageBox::information(this, "Info", "Nodes imported successfully!");
    } catch (const std::exception &e) {
        QMessageBox::critical(this, "Error", "Import nodes failed: " + QString(e.what()));
    }
}

void RobotControl::import_nodes(std::string fileName)
{
    if (fileName.empty()) return;

    std::ifstream ifs(fileName);
    if (!ifs.is_open()) {
        throw std::runtime_error("Failed to open file: " + fileName);
    }

    nlohmann::json list;
    ifs >> list;
    ifs.close();

    int size = list.size();
    ui_.comboBox->clear();
    nodeList.clear();
    line_lists.clear();

    for (int i = 0; i < size; ++i)
    {
        std::string name = list[i]["name"].get<std::string>();
        nodeList[name].name = name;
        nodeList[name].pose = json_to_pose(list[i]);
        nodeList[name].cross_type = list[i]["cross_type"].get<uint32_t>();
        nodeList[name].putter_hight = list[i]["putter_hight"].get<uint32_t>();
        nodeList[name].is_charge = list[i]["is_charge"].get<bool>();
        nodeList[name].dir_type = list[i]["dir_type"].get<uint32_t>();

        for (auto &conn : list[i]["connect_node"]) {
            nodeList[name].connect_node.push_back(conn.get<std::string>());
            line_lists.emplace_back(name, conn.get<std::string>());
        }

        ui_.comboBox->addItem(QString::fromStdString(name));
    }

    pub_node_list();
}

void RobotControl::import_nodes(void)
{
    QString fileName = "/install/nodes/my.json";
    if (fileName.isEmpty()) return;

    std::ifstream ifs(fileName.toStdString());
    if (!ifs.is_open()) {
        throw std::runtime_error("Failed to open file: " + fileName.toStdString());
    }

    nlohmann::json list;
    ifs >> list;
    ifs.close();

    int size = list["data"].size();
    ui_.comboBox->clear();
    nodeList.clear();
    line_lists.clear();

    for (int i = 0; i < size; ++i)
    {
        std::string name = list["data"][i]["navigation_id"].get<std::string>();
        nodeList[name].name = name;
        nodeList[name].pose.position.x = list["data"][i]["point_x"].get<float>();
        nodeList[name].pose.position.y = list["data"][i]["point_y"].get<float>();
        
        double yaw = list["data"][i]["point_theta"].get<float>();
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        nodeList[name].pose.orientation = tf2::toMsg(q);

        ui_.comboBox->addItem(QString::fromStdString(name));
    }

    pub_node_list();
}

void RobotControl::import_edges(void)
{
    QString fileName = "/install/nodes/robot_navigation_edges.json";
    if (fileName.isEmpty()) return;

    std::ifstream ifs(fileName.toStdString());
    if (!ifs.is_open()) {
        throw std::runtime_error("Failed to open file: " + fileName.toStdString());
    }

    nlohmann::json list;
    ifs >> list;
    ifs.close();

    int size = list["data"].size();
    line_lists.clear();

    for (int i = 0; i < size; ++i)
    {
        std::string start = list["data"][i]["start_id"].get<std::string>();
        std::string end = list["data"][i]["end_id"].get<std::string>();
        nodeList[start].connect_node.push_back(end);
        nodeList[end].connect_node.push_back(start);
        line_lists.emplace_back(start, end);
    }

    pub_node_list();
}

void RobotControl::on_saveNode_clicked()
{
    QString fileName = QFileDialog::getSaveFileName(
        this, "Save File", "/install/nodes", "JSON File(*.json)");
    if (fileName.isEmpty()) return;

    try {
        nlohmann::json root;
        for (auto &it : nodeList)
        {
            nlohmann::json node;
            node["name"] = it.first;
            node["pose"] = convert_pose_to_json(it.second.pose);
            node["cross_type"] = it.second.cross_type;
            node["putter_hight"] = it.second.putter_hight;
            node["is_charge"] = it.second.is_charge;
            node["dir_type"] = it.second.dir_type;
            
            for (auto &conn : it.second.connect_node) {
                node["connect_node"].push_back(conn);
            }
            root.push_back(node);
        }

        std::string save_path = fileName.toStdString();
        if (save_path.find(".json") == std::string::npos) {
            save_path += ".json";
        }
        std::ofstream ofs(save_path);
        if (!ofs.is_open()) {
            throw std::runtime_error("Failed to open save file: " + save_path);
        }
        ofs << root.dump(4);
        ofs.close();

        QMessageBox::information(this, "Info", "Nodes saved to: " + QString::fromStdString(save_path));
    } catch (const std::exception &e) {
        QMessageBox::critical(this, "Error", "Save nodes failed: " + QString(e.what()));
    }
}

void RobotControl::on_comboBox_currentIndexChanged(const QString &arg1)
{
    try {
        std::string name = arg1.toStdString();
        ui_.putterTaskHeight->setValue(nodeList[name].putter_hight);
        ui_.crossBox->setCurrentIndex(nodeList[name].cross_type);
    } catch (const std::exception &e) {
        RCLCPP_WARN(nh_->get_logger(), "ComboBox change error: %s", e.what());
    }
}

void RobotControl::on_stopNavigation_clicked()
{
    robot_task_msgs::msg::RobotNavigationCmd navigationStop;
    navigationStop.cmd_type = 0;
    pubNavigationStart->publish(navigationStop);
    QMessageBox::information(this, "Info", "Navigation stopped!");
}

void RobotControl::on_startNavigation_clicked()
{
    robot_task_msgs::msg::RobotNavigationCmd navigationStart;
    navigationStart.cmd_type = 1;
    pubNavigationStart->publish(navigationStart);
    QMessageBox::information(this, "Info", "Navigation started!");
}

void RobotControl::on_clearPath_clicked()
{
    robot_task_msgs::msg::RobotNavigationCmd navigationClear;
    navigationClear.cmd_type = 2;
    pubNavigationStart->publish(navigationClear);
    QMessageBox::information(this, "Info", "Path cleared!");
}

void RobotControl::on_crossUp_clicked()
{
    cmd.task_type = 0x0D;
    cmd.cross.type = 1;
    pubRobotCmd->publish(cmd);
    QMessageBox::information(this, "Info", "Cross up command sent!");
}

void RobotControl::on_crossDown_clicked()
{
    cmd.task_type = 0x0D;
    cmd.cross.type = 2;
    pubRobotCmd->publish(cmd);
    QMessageBox::information(this, "Info", "Cross down command sent!");
}

void RobotControl::on_crossSpeedSet_clicked()
{
    cmd.task_type = 0x0E;
    cmd.putter.hight = ui_.crossSpeed->value();
    pubRobotCmd->publish(cmd);
    QMessageBox::information(this, "Info", "Cross speed set to: " + QString::number(ui_.crossSpeed->value()));
}

void RobotControl::on_startCharge_clicked()
{
    robot_task_msgs::msg::RobotNavigationCmd navigationStart;
    navigationStart.cmd_type = 3;
    pubNavigationStart->publish(navigationStart);
    QMessageBox::information(this, "Info", "Charging started!");
}

void RobotControl::on_stopCharge_clicked()
{
    robot_task_msgs::msg::RobotNavigationCmd navigationStart;
    navigationStart.cmd_type = 4;
    pubNavigationStart->publish(navigationStart);
    QMessageBox::information(this, "Info", "Charging stopped!");
}

void RobotControl::on_setCharge_clicked()
{
    try {
        std::string name = ui_.comboBox->currentText().toStdString();
        for (auto &it : nodeList)
        {
            if (it.second.name == name)
                it.second.is_charge = !it.second.is_charge;
            else
                it.second.is_charge = false;
        }
        pub_node_list();
        QMessageBox::information(this, "Info", "Charge point updated!");
    } catch (const std::exception &e) {
        QMessageBox::critical(this, "Error", "Set charge point failed: " + QString(e.what()));
    }
}

void RobotControl::on_agvInit_clicked()
{
    cmd.task_type = 0x04;
    pubRobotCmd->publish(cmd);
    QMessageBox::information(this, "Info", "AGV initialized!");
}

void RobotControl::on_ConnectSet_clicked()
{
    deleteFlag = false;
    firstDeleteFlag = false;
    static QString color = ui_.ConnectSet->styleSheet();

    if (!connectFlag)
    {
        ui_.ConnectSet->setText("ConnectEnd");
        ui_.ConnectSet->setStyleSheet("background-color: rgb(175,238,238)");
        QMessageBox::information(this, "Info", "Connect mode started! Click two nodes to connect.");
    }
    else
    {
        ui_.ConnectSet->setText("ConnectStart");
        ui_.ConnectSet->setStyleSheet(color);
        QMessageBox::information(this, "Info", "Connect mode stopped!");
    }
    connectFlag = !connectFlag;
    if (connectFlag)
        firstConnectFlag = true;
}

void RobotControl::on_deleteSet_clicked()
{
    connectFlag = false;
    firstConnectFlag = false;
    static QString color = ui_.deleteSet->styleSheet();

    if (!deleteFlag)
    {
        ui_.deleteSet->setText("DeleteEnd");
        ui_.deleteSet->setStyleSheet("background-color: rgb(175,238,238)");
        QMessageBox::information(this, "Info", "Delete mode started! Click two nodes to disconnect.");
    }
    else
    {
        ui_.deleteSet->setText("DeleteStart");
        ui_.deleteSet->setStyleSheet(color);
        QMessageBox::information(this, "Info", "Delete mode stopped!");
    }
    deleteFlag = !deleteFlag;
    if (deleteFlag)
        firstDeleteFlag = true;
}

void RobotControl::on_setDirection_clicked()
{
    try {
        std::string name = ui_.comboBox->currentText().toStdString();
        nodeList[name].dir_type = (nodeList[name].dir_type == 0) ? 1 : 0;
        pub_node_list();
        QMessageBox::information(this, "Info", "Node direction toggled!");
    } catch (const std::exception &e) {
        QMessageBox::critical(this, "Error", "Set direction failed: " + QString(e.what()));
    }
}

} // namespace robot_control

// 插件注册
PLUGINLIB_EXPORT_CLASS(robot_control::RobotControl, rviz_common::Panel)