
#include <stdio.h>

#include "../include/remote_control/remote_control.h"
#include <iostream>
#include <fstream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace remote_control
{

  RemoteControl::RemoteControl(ros::NodeHandle nh)
  {
    nh_ = nh;
    ros::NodeHandle nh_p("~");
    nh_p.param<std::string>("node_file", node_file, "");
    pubKeepTask = nh_.advertise<robot_task_msgs::robot_keep_task_send>("/keep_task_send", 1);
    pubOneTask = nh_.advertise<robot_task_msgs::robot_one_task_send>("/one_task_send", 1);
    pubRobotCmd = nh_.advertise<robot_state_msgs::data_to_stm32>("data_to_stm32", 2);
    pubGoalSend = nh_.advertise<robot_task_msgs::robot_goal>("/goal_node", 1);
    pubNavigationStart = nh_.advertise<robot_task_msgs::robot_navigation_cmd>("/navigation_set", 1);
    pubNodeList = nh_.advertise<visualization_msgs::MarkerArray>("/node_list", 1, true);
    pubGoalList = nh_.advertise<robot_task_msgs::robot_goal_list>("/goal_list", 1);

    subRobotPose = nh_.subscribe("/robot_pose", 1, &RemoteControl::RobotPoseCallBack, this);
    subRobotState = nh_.subscribe("/robot_state", 1, &RemoteControl::StateCallBack, this);
    subClickPoint = nh_.subscribe("/clicked_point", 1, &RemoteControl::ClickPointCallBack, this);
    subRemoteCmd = nh_.subscribe("/remote_cmd", 1, &RemoteControl::RemoteCmdCallBack, this);

    moveTimer = nh_.createTimer(ros::Duration(0.2), &RemoteControl::CmdSet, this);
    // import_nodes();
    // import_edges();
    import_nodes(node_file);
  }

  Json::Value RemoteControl::convert_pose_to_json(geometry_msgs::Pose pose)
  {
    Json::Value node;
    Json::Value position;
    Json::Value orientation;

    position["x"] = pose.position.x;
    position["y"] = pose.position.y;
    position["z"] = pose.position.z;
    orientation["x"] = pose.orientation.x;
    orientation["y"] = pose.orientation.y;
    orientation["z"] = pose.orientation.z;
    orientation["w"] = pose.orientation.w;
    node["position"] = position;
    node["orientation"] = orientation;
    return node;
  }

  geometry_msgs::Pose RemoteControl::convert_pose_to_json(Json::Value &list)
  {
    geometry_msgs::Pose pose;
    pose.position.x = list["pose"]["position"]["x"].asFloat();
    pose.position.y = list["pose"]["position"]["y"].asFloat();
    pose.position.z = list["pose"]["position"]["z"].asFloat();

    pose.orientation.x = list["pose"]["orientation"]["x"].asFloat();
    pose.orientation.y = list["pose"]["orientation"]["y"].asFloat();
    pose.orientation.z = list["pose"]["orientation"]["z"].asFloat();
    pose.orientation.w = list["pose"]["orientation"]["w"].asFloat();
    return pose;
  }

  visualization_msgs::Marker RemoteControl::convert_node_to_mark(int id,
                                                                 std::string name,
                                                                 geometry_msgs::Pose &pose,
                                                                 int type)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id;
    if (type == 0)
    {
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.scale.z = 0.2;
      marker.color.b = 255;
      marker.color.g = 255;
      marker.color.r = 0;
      marker.color.a = 1;
      marker.text = name;
      marker.pose = pose;
    }
    else if (type == 1)
    {
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;
      marker.color.b = 0;
      marker.color.g = 170;
      marker.color.r = 250;
      marker.color.a = 0.5;
      marker.pose = pose;
    }
    else if (type == 2)
    {
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.scale.x = 0.4;
      marker.scale.y = 0.4;
      marker.scale.z = 0.4;
      marker.color.b = 0;
      marker.color.g = 0;
      marker.color.r = 250;
      marker.color.a = 0.5;
      marker.pose = pose;
    }
    else if (type == 3)
    {
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;
      marker.color.b = 0;
      marker.color.g = 250;
      marker.color.r = 0;
      marker.color.a = 0.5;
      marker.pose = pose;
    }

    return marker;
  }

  void RemoteControl::connect_two_points(visualization_msgs::Marker &line_list, geometry_msgs::Pose &p1, geometry_msgs::Pose &p2)
  {
    line_list.header.frame_id = "map";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "basic_shapes";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.025;
    line_list.color.b = 25;
    line_list.color.g = 0;
    line_list.color.r = 20;
    line_list.color.a = 0.5;
    geometry_msgs::Point p;
    p.x = p1.position.x;
    p.y = p1.position.y;
    p.z = p1.position.z;
    line_list.points.push_back(p);
    p.x = p2.position.x;
    p.y = p2.position.y;
    p.z = p2.position.z;
    line_list.points.push_back(p);
  }

  void RemoteControl::pub_node_list(void)
  {
    visualization_msgs::MarkerArray markerArray;
    robot_task_msgs::robot_goal_list goalList;
    markerArray.markers.clear();
    pubNodeList.publish(markerArray);

    int i = 0;
    for (std::map<std::string, robot_task_msgs::robot_goal>::iterator it = nodeList.begin(); it != nodeList.end(); ++it)
    {
      std::string name = it->first;
      geometry_msgs::Pose pose = it->second.pose;
      markerArray.markers.push_back(convert_node_to_mark(i++, name, pose, 0));
      markerArray.markers.push_back(convert_node_to_mark(i++, name, pose, it->second.is_charge ? 2 : (it->second.dir_type == 0 ? 1 : 3)));
      goalList.node.push_back(it->second);
    }

    visualization_msgs::Marker lines;
    if (!line_lists.empty())
    {
      for (int i = 0; i < line_lists.size(); i++)
        connect_two_points(lines, nodeList[line_lists[i].first].pose, nodeList[line_lists[i].second].pose);
      markerArray.markers.push_back(lines);
    }

    pubGoalList.publish(goalList);
    pubNodeList.publish(markerArray);
  }

  void RemoteControl::CmdSet(const ros::TimerEvent &event)
  {
    if (robotState.current_task == RobotCharging)
      robotMoveFlag = false;

    if (robotMoveFlag)
      pubRobotCmd.publish(cmd);

    if (robotChargeFlag && (robotState.current_task == RobotIdle || robotState.current_task == RobotRadiusRunning))
    {
      if (move_count < 25)
      {
        move_count++;
        pubRobotCmd.publish(cmd);
      }
      else
      {
        robotChargeFlag = false;
        move_count = 0;
        cmd.running.speed = 0;
        pubRobotCmd.publish(cmd);
      }
    }
  }

  void RemoteControl::RobotPoseCallBack(const nav_msgs::OdometryConstPtr &pose)
  {
    boost::unique_lock<boost::shared_mutex> lockDeal(mutexReadPose);
    robotPose = *pose;
    // ui_.robot_pose_x->setValue(robotPose.pose.pose.position.x);
    // ui_.robot_pose_y->setValue(robotPose.pose.pose.position.y);
  }

  void RemoteControl::StateCallBack(const robot_state_msgs::robot_stateConstPtr data)
  {
    // ui_.curTask->setValue(data->current_task);
    // ui_.battery->setValue(data->battery_capacity > 100 ? 100 : data->battery_capacity);
    // ui_.warnning->setValue(data->warning_code);
    // ui_.erroCode->setValue(data->error_code);
    // ui_.ultrasonic->setValue(data->ultrasonicValue);
    // ui_.chargeState->setValue(data->charge_state);
    // ui_.speed->setValue(data->speed);
    // ui_.radius->setValue(data->radius);
    // ui_.putterHeight_2->setValue(data->putter_height);
    robotState = *data;
  }

  void RemoteControl::RemoteCmdCallBack(const std_msgs::HeaderConstPtr cmd)
  {
  }

  void RemoteControl::ClickPointCallBack(const geometry_msgs::PointStampedConstPtr point)
  {
    if (connectFlag)
    {
      for (std::map<std::string, robot_task_msgs::robot_goal>::iterator it = nodeList.begin(); it != nodeList.end(); ++it)
      {
        if (hypot(it->second.pose.position.x - point->point.x, it->second.pose.position.y - point->point.y) < 0.1)
        {
          std::pair<std::string, std::string> connectPoint;
          if (line_lists.empty() || firstConnectFlag)
          {
            connectPoint = std::make_pair(it->first, it->first);
            firstConnectFlag = false;
          }
          else
          {
            connectPoint = std::make_pair(line_lists.at(line_lists.size() - 1).second, it->first);
            nodeList[it->first].connect_node.push_back(line_lists.at(line_lists.size() - 1).second);
            nodeList[line_lists.at(line_lists.size() - 1).second].connect_node.push_back(it->first);
          }
          line_lists.push_back(connectPoint);
          break;
        }
      }
    }

    if (deleteFlag)
    {
      static std::string firstNode;
      for (std::map<std::string, robot_task_msgs::robot_goal>::iterator it = nodeList.begin(); it != nodeList.end(); ++it)
      {
        if (hypot(it->second.pose.position.x - point->point.x, it->second.pose.position.y - point->point.y) < 0.1)
        {
          if (firstDeleteFlag)
          {
            firstNode = it->first;
            firstDeleteFlag = false;
            break;
          }
          else
          {
            disconnect_two_points(firstNode, it->first);
            firstNode = it->first;
            break;
          }
        }
      }
    }

    pub_node_list();
  }

  void RemoteControl::disconnect_two_points(std::string first, std::string second)
  {
    for (int i = 0; i < nodeList[first].connect_node.size(); i++)
    {
      if (nodeList[first].connect_node[i] == second)
      {
        nodeList[first].connect_node.erase(nodeList[first].connect_node.begin() + i);
        break;
      }
    }
    for (int i = 0; i < nodeList[second].connect_node.size(); i++)
    {
      if (nodeList[second].connect_node[i] == first)
      {
        nodeList[second].connect_node.erase(nodeList[second].connect_node.begin() + i);
        break;
      }
    }

    line_lists.clear();
    for (std::map<std::string, robot_task_msgs::robot_goal>::iterator it = nodeList.begin(); it != nodeList.end(); ++it)
    {
      for (int j = 0; j < it->second.connect_node.size(); ++j)
      {
        std::pair<std::string, std::string> connectPoint;
        connectPoint.first = it->first;
        connectPoint.second = it->second.connect_node[j];
        line_lists.push_back(connectPoint);
      }
    }
  }

  /*
  void RemoteControl::on_MapBuildStart_clicked(std_msgs::Header &cmd_remote)
  {
    robot_task_msgs::robot_keep_task_send send;
    send.type = MAP_BUILD;
    pubKeepTask.publish(send);
    // ui_.MapBuildStart->setEnabled(false);
    // ui_.SaveMap->setEnabled(true);
    // ui_.MapBUildStop->setEnabled(true);
    // ui_.LocationStart->setEnabled(false);
    // ui_.LocationStop->setEnabled(false);
  }

  void RemoteControl::on_SaveMap_clicked(std_msgs::Header &cmd_remote)
  {
    robot_task_msgs::robot_one_task_send send;
    send.type = MAP_SAVE;
    pubOneTask.publish(send);
    // ui_.MapBuildStart->setEnabled(false);
    // ui_.SaveMap->setEnabled(false);
    // ui_.MapBUildStop->setEnabled(true);
    // ui_.LocationStart->setEnabled(false);
    // ui_.LocationStop->setEnabled(false);
  }

  void RemoteControl::on_MapBUildStop_clicked(std_msgs::Header &cmd_remote)
  {
    robot_task_msgs::robot_one_task_send send;
    send.type = MAP_STOP_BUILD;
    pubOneTask.publish(send);
    // ui_.MapBUildStop->setEnabled(false);
    // sleep(2);
    // ui_.MapBuildStart->setEnabled(true);
    // ui_.SaveMap->setEnabled(false);
    // ui_.LocationStart->setEnabled(true);
    // ui_.LocationStop->setEnabled(false);
  }

  void RemoteControl::on_LocationStart_clicked(std_msgs::Header &cmd_remote)
  {
    robot_task_msgs::robot_keep_task_send send;
    send.type = LOCATION_START;
    pubKeepTask.publish(send);
    // ui_.MapBuildStart->setEnabled(false);
    // ui_.SaveMap->setEnabled(false);
    // ui_.MapBUildStop->setEnabled(false);
    // ui_.LocationStart->setEnabled(false);
    // ui_.LocationStop->setEnabled(true);
  }

  void RemoteControl::on_LocationStop_clicked(std_msgs::Header &cmd_remote)
  {
    robot_task_msgs::robot_one_task_send send;
    send.type = LOCATION_STOP;
    pubOneTask.publish(send);
    // ui_.LocationStop->setEnabled(false);
    // sleep(2);
    // ui_.MapBuildStart->setEnabled(true);
    // ui_.SaveMap->setEnabled(false);
    // ui_.MapBUildStop->setEnabled(false);
    // ui_.LocationStart->setEnabled(true);
  }

  */
}

void remote_control::RemoteControl::on_robot_go_pressed()
{
  cmd.task_type = 2;
  cmd.running.type = 1;
  cmd.running.speed = 0.1;
  cmd.running.radius = 0;
  robotMoveFlag = true;
}

void remote_control::RemoteControl::on_robot_back_pressed()
{
  cmd.task_type = 2;
  cmd.running.type = 1;
  cmd.running.speed = -0.1;
  cmd.running.radius = 0;

  robotMoveFlag = true;
}

void remote_control::RemoteControl::on_robot_right_pressed()
{
  cmd.task_type = 2;
  cmd.running.type = 1;
  cmd.running.speed = 0;
  cmd.running.radius = 0.1;

  robotMoveFlag = true;
}

void remote_control::RemoteControl::on_robot_left_pressed()
{
  cmd.task_type = 2;
  cmd.running.type = 1;
  cmd.running.speed = 0;
  cmd.running.radius = -0.1;

  robotMoveFlag = true;
}

void remote_control::RemoteControl::on_robot_go_released()
{
  robotMoveFlag = false;
}

void remote_control::RemoteControl::on_robot_back_released()
{
  robotMoveFlag = false;
}

void remote_control::RemoteControl::on_robot_left_released()
{
  robotMoveFlag = false;
}

void remote_control::RemoteControl::on_robot_right_released()
{
  robotMoveFlag = false;
}

/**************************************************************/

void remote_control::RemoteControl::on_setGoal_clicked(std_msgs::Header &cmd_remote)
{
  robot_task_msgs::robot_goal goal;
  goal.name = cmd_remote.frame_id;
  goal.pose = nodeList[goal.name].pose;
  // goal.putter_hight = ui_.putterTaskHeight->value();
  // goal.cross_type = ui_.crossBox->currentIndex();
  // ui_.goal_pose_x->setValue(goal.pose.position.x);
  // ui_.goal_pose_y->setValue(goal.pose.position.y);

  pubGoalSend.publish(goal);
}

void remote_control::RemoteControl::on_insertNode_clicked(std_msgs::Header &cmd_remote)
{
  std::string node_name = cmd_remote.frame_id;
  if (node_name == "")
  {
    ROS_WARN("Node name cannot be empty!");
    return;
  }

  nodeList[node_name].pose = robotPose.pose.pose;
  pub_node_list();
}

void remote_control::RemoteControl::on_nodeClear_clicked(std_msgs::Header &cmd_remote)
{
  nodeList.clear();
  line_lists.clear();
  pub_node_list();
}

void remote_control::RemoteControl::on_importNode_clicked(std_msgs::Header &cmd_remote)
{
  import_nodes(cmd_remote.frame_id);
}

void remote_control::RemoteControl::import_nodes(std::string fileName)
{
  if (fileName == "")
  {
    ROS_WARN("fileName is empty!");
    return;
  }
  else
  {
    std::ifstream ifs;
    ifs.open(fileName);
    assert(ifs.is_open());

    Json::Reader reader;
    Json::Value list;

    if (!reader.parse(ifs, list, false))
    {
      return;
    }
    int size = list.size();

    // ui_.comboBox->clear();
    nodeList.clear();
    line_lists.clear();
    for (int i = 0; i < size; ++i)
    {
      std::string name = list[i]["name"].asString();
      nodeList[name].name = name;
      nodeList[name].pose = convert_pose_to_json(list[i]);
      nodeList[name].cross_type = list[i]["cross_type"].asUInt();
      nodeList[name].putter_hight = list[i]["putter_hight"].asUInt();
      nodeList[name].is_charge = list[i]["is_charge"].asBool();
      nodeList[name].dir_type = list[i]["dir_type"].asUInt();
      for (int j = 0; j < list[i]["connect_node"].size(); ++j)
      {
        nodeList[name].connect_node.push_back(list[i]["connect_node"][j].asString());
        std::pair<std::string, std::string> connectPoint;
        connectPoint.first = name;
        connectPoint.second = list[i]["connect_node"][j].asString();
        line_lists.push_back(connectPoint);
      }
      nodeList[name].putter_hight = list[i]["putter_hight"].asUInt();
      // ui_.comboBox->addItem(QString::fromStdString(name));
    }
    ROS_INFO("node num: %ld", size);
    pub_node_list();
  }
}

void remote_control::RemoteControl::import_nodes(void)
{
  std::string fileName; //= workspace_path + "/install/nodes/my.json";
  if (fileName == "")
  {
    return;
  }
  else
  {
    std::ifstream ifs;
    ifs.open(fileName);
    assert(ifs.is_open());

    Json::Reader reader;
    Json::Value list;

    if (!reader.parse(ifs, list, false))
    {
      return;
    }
    int size = list["data"].size();

    // ui_.comboBox->clear();
    // nodeList.clear();
    // line_lists.clear();
    // for (int i = 0; i < size; ++i)
    // {
    //   std::string name = list["data"][i]["navigation_id"].asString();
    //   nodeList[name].name = name;
    //   nodeList[name].pose.position.x = list["data"][i]["point_x"].asFloat();
    //   nodeList[name].pose.position.y = list["data"][i]["point_y"].asFloat();
    //   nodeList[name].pose.orientation = tf::createQuaternionMsgFromYaw(list["data"][i]["point_theta"].asFloat());
    //   ui_.comboBox->addItem(QString::fromStdString(name));
    // }

    pub_node_list();
  }
}

void remote_control::RemoteControl::import_edges(void)
{
  // QString fileName = workspace_path + "/install/nodes/robot_navigation_edges.json";
  // if (fileName == "")
  // {
  //   return;
  // }
  // else
  // {
  //   std::ifstream ifs;
  //   ifs.open(fileName.toStdString());
  //   assert(ifs.is_open());

  //   Json::Reader reader;
  //   Json::Value list;

  //   if (!reader.parse(ifs, list, false))
  //   {
  //     return;
  //   }
  //   int size = list["data"].size();

  //   line_lists.clear();
  //   for (int i = 0; i < size; ++i)
  //   {
  //     std::string start = list["data"][i]["start_id"].asString();
  //     std::string end = list["data"][i]["end_id"].asString();
  //     nodeList[start].connect_node.push_back(end);
  //     nodeList[end].connect_node.push_back(start);
  //     std::pair<std::string, std::string> connectPoint;
  //     connectPoint.first = start;
  //     connectPoint.second = end;
  //     line_lists.push_back(connectPoint);
  //   }

  //   pub_node_list();
  // }
}

void remote_control::RemoteControl::on_saveNode_clicked(std_msgs::Header &cmd_remote)
{
  std::string fileName = cmd_remote.frame_id;
  if (fileName == "")
  {
    return;
  }

  Json::FastWriter writer;
  Json::Value root;

  for (std::map<std::string, robot_task_msgs::robot_goal>::iterator it = nodeList.begin(); it != nodeList.end(); ++it)
  {
    Json::Value node;
    node["name"] = it->first;
    node["pose"] = convert_pose_to_json(it->second.pose);
    node["cross_type"] = it->second.cross_type;
    node["putter_hight"] = it->second.putter_hight;
    node["is_charge"] = it->second.is_charge;
    node["dir_type"] = it->second.dir_type;
    for (int i = 0; i < it->second.connect_node.size(); i++)
    {
      node["connect_node"].append(it->second.connect_node[i]);
    }
    root.append(node);
  }
  std::string json_file = writer.write(root);

  std::ofstream ofs;
  ofs.open(fileName + ".json");
  assert(ofs.is_open());
  ofs << json_file;
}

void remote_control::RemoteControl::on_comboBox_currentIndexChanged()
{
  // ui_.putterTaskHeight->setValue(nodeList[arg1.toStdString()].putter_hight);
  // ui_.crossBox->setCurrentIndex(nodeList[arg1.toStdString()].cross_type);
}

void remote_control::RemoteControl::on_stopNavigation_clicked(std_msgs::Header &cmd_remote)
{
  robot_task_msgs::robot_navigation_cmd navigationStop;
  navigationStop.cmd_type = 0;
  pubNavigationStart.publish(navigationStop);
}

void remote_control::RemoteControl::on_startNavigation_clicked(std_msgs::Header &cmd_remote)
{
  robot_task_msgs::robot_navigation_cmd navigationStart;
  navigationStart.cmd_type = 1;
  pubNavigationStart.publish(navigationStart);
}

void remote_control::RemoteControl::on_clearPath_clicked(std_msgs::Header &cmd_remote)
{
  robot_task_msgs::robot_navigation_cmd navigationClear;
  navigationClear.cmd_type = 2;
  pubNavigationStart.publish(navigationClear);
}

/**************************************************************/

// void remote_control::RemoteControl::on_putterSet_clicked(std_msgs::Header &cmd_remote)
//{
//   cmd.task_type = 0x0E;
//   //cmd.putter.hight = ui_.putterHeight->value();
//   pubRobotCmd.publish(cmd);
// }

// void remote_control::RemoteControl::on_putterInit_clicked(std_msgs::Header &cmd_remote)
//{
//   cmd.task_type = 0x0F;
//   pubRobotCmd.publish(cmd);
// }

void remote_control::RemoteControl::on_crossUp_clicked(std_msgs::Header &cmd_remote)
{
  cmd.task_type = 0x0D;
  cmd.cross.type = 1;
  pubRobotCmd.publish(cmd);
}

void remote_control::RemoteControl::on_crossDown_clicked(std_msgs::Header &cmd_remote)
{
  cmd.task_type = 0x0D;
  cmd.cross.type = 2;
  pubRobotCmd.publish(cmd);
}

void remote_control::RemoteControl::on_crossSpeedSet_clicked(std_msgs::Header &cmd_remote)
{
  // cmd.task_type = 0x0D;
  // cmd.cross.type = 0;
  // cmd.cross.speed = ui_.crossSpeed->value() / 1000.0f;
  // pubRobotCmd.publish(cmd);
  cmd.task_type = 0x0E;
  // cmd.putter.hight = std::(cmd_remote.frame_id);
  pubRobotCmd.publish(cmd);
}

void remote_control::RemoteControl::on_startCharge_clicked(std_msgs::Header &cmd_remote)
{
  robot_task_msgs::robot_navigation_cmd navigationStart;
  navigationStart.cmd_type = 3;
  pubNavigationStart.publish(navigationStart);
}

void remote_control::RemoteControl::on_stopCharge_clicked(std_msgs::Header &cmd_remote)
{
  robot_task_msgs::robot_navigation_cmd navigationStart;
  navigationStart.cmd_type = 4;
  pubNavigationStart.publish(navigationStart);
}

void remote_control::RemoteControl::on_setCharge_clicked(std_msgs::Header &cmd_remote)
{
  std::string name = cmd_remote.frame_id; // ui_.comboBox->currentText().toStdString();
  for (std::map<std::string, robot_task_msgs::robot_goal>::iterator it = nodeList.begin(); it != nodeList.end(); ++it)
  {
    if (it->second.name == name)
      it->second.is_charge = !it->second.is_charge;
    else
      it->second.is_charge = false;
  }
  pub_node_list();
}

void remote_control::RemoteControl::on_agvInit_clicked(std_msgs::Header &cmd_remote)
{
  cmd.task_type = 0x04;
  pubRobotCmd.publish(cmd);
}

void remote_control::RemoteControl::on_ConnectSet_clicked(std_msgs::Header &cmd_remote)
{
  deleteFlag = false;
  firstDeleteFlag = false;

  connectFlag = !connectFlag;
  if (connectFlag)
    firstConnectFlag = true;
}

void remote_control::RemoteControl::on_deleteSet_clicked(std_msgs::Header &cmd_remote)
{
  connectFlag = false;
  firstConnectFlag = false;

  deleteFlag = !deleteFlag;
  if (deleteFlag)
    firstDeleteFlag = true;
}

void remote_control::RemoteControl::on_setDirection_clicked(std_msgs::Header &cmd_remote)
{
  std::string name = cmd_remote.frame_id;
  nodeList[name].dir_type = nodeList[name].dir_type == 0 ? 1 : 0;
  pub_node_list();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "remote_control_node");
  ros::NodeHandle n;
  remote_control::RemoteControl node(n);
  ros::spin();
  return 0;
}