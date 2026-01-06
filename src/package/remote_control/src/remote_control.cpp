
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
    pubRobotCmd = nh_.advertise<robot_state_msgs::data_to_stm32>("data_to_stm32", 2);
    pubGoalSend = nh_.advertise<robot_task_msgs::robot_goal>("/goal_node", 1);
    pubNavigationStart = nh_.advertise<robot_task_msgs::robot_navigation_cmd>("/navigation_set", 1);
    pubNodeList = nh_.advertise<visualization_msgs::MarkerArray>("/node_list", 1, true);
    pubGoalList = nh_.advertise<robot_task_msgs::robot_goal_list>("/goal_list", 1);
    pubDebugLaser = nh_.advertise<std_msgs::Int8>("/debug_cmd", 1);

    subRobotPose = nh_.subscribe("/robot_pose", 1, &RemoteControl::RobotPoseCallBack, this);
    subClickPoint = nh_.subscribe("/clicked_point", 1, &RemoteControl::ClickPointCallBack, this);
    subRemoteCmd = nh_.subscribe("/remote_cmd", 1, &RemoteControl::RemoteCmdCallBack, this);
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

    int i = 0;
    for (auto &node_pair : nodeList)
    {
      std::string name = node_pair.first;
      geometry_msgs::Pose pose = node_pair.second.pose;
      markerArray.markers.push_back(convert_node_to_mark(i++, name, pose, 0));
      markerArray.markers.push_back(convert_node_to_mark(i++, name, pose, node_pair.second.is_charge ? 2 : (node_pair.second.dir_type == 0 ? 1 : 3)));
      goalList.node.push_back(node_pair.second);
    }

    visualization_msgs::Marker lines;
    lines.header.frame_id = "map";
    lines.header.stamp = ros::Time::now();
    lines.ns = "basic_shapes";
    lines.action = visualization_msgs::Marker::ADD;
    lines.id = 2;
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.scale.x = 0.025;
    lines.color.b = 25.0 / 255.0;
    lines.color.g = 0.0;
    lines.color.r = 20.0 / 255.0;
    lines.color.a = 0.5;
    lines.points.clear();

    if (!line_lists.empty())
    {
      for (auto &line_pair : line_lists)
      {
        std::string node_name1 = line_pair.first;
        std::string node_name2 = line_pair.second;

        auto node_it1 = nodeList.find(node_name1);
        auto node_it2 = nodeList.find(node_name2);

        if (node_it1 != nodeList.end() && node_it2 != nodeList.end())
        {
          geometry_msgs::Pose &p1 = node_it1->second.pose;
          geometry_msgs::Pose &p2 = node_it2->second.pose;
          connect_two_points(lines, p1, p2);
        }
      }

      if (!lines.points.empty())
      {
        markerArray.markers.push_back(lines);
      }
    }

    pubGoalList.publish(goalList);
    pubNodeList.publish(markerArray);
  }

  void RemoteControl::RobotPoseCallBack(const geometry_msgs::PoseStampedConstPtr &pose)
  {
    boost::unique_lock<boost::shared_mutex> lockDeal(mutexReadPose);
    robotPose = *pose;
  }

  void RemoteControl::RemoteCmdCallBack(const std_msgs::HeaderConstPtr cmd_control)
  {
    uint32_t seq = cmd_control->seq;
    std_msgs::Header cmd_copy = *cmd_control;

    switch (seq)
    {
    case NoCmd:
      break;
    case AGVInit:
      on_agvInit_clicked(cmd_copy);
      break;
    case CorssUpLeft:
    case CorssUpRight:
    case CorssDownLeft:
    case CorssDownRight:
      on_cross_clicked(cmd_copy);
      break;
    case PutterSet:
      on_crossSpeedSet_clicked(cmd_copy);
      break;
    case SetPathNode:
      on_setGoal_clicked(cmd_copy);
      break;
    case StartNavigation:
      on_startNavigation_clicked(cmd_copy);
      break;
    case StopNavigation:
      on_stopNavigation_clicked(cmd_copy);
      break;
    case InsertNode:
      on_insertNode_clicked(cmd_copy);
      break;
    case ConnectStart:
      on_ConnectSet_clicked(cmd_copy);
      break;
    case DeleteStart:
      on_deleteSet_clicked(cmd_copy);
      break;
    case ImportNodes:
      on_importNode_clicked(cmd_copy);
      break;
    case SaveNodes:
      on_saveNode_clicked(cmd_copy);
      break;
    case SetDir:
      on_setDirection_clicked(cmd_copy);
      break;
    case SetCharge:
      on_setCharge_clicked(cmd_copy);
      break;
    case StartCharge:
      on_startCharge_clicked(cmd_copy);
      break;
    case StopCharge:
      on_stopCharge_clicked(cmd_copy);
      break;
    case 19:
      on_deleteNode_clicked(cmd_copy);
      break;
    case DebugLaser:
      on_debugLaser_clicked(cmd_copy);
      break;
    default:
      ROS_WARN("RemoteCmdCallBack: unknown seq %u (frame_id=%s)", seq, cmd_copy.frame_id.c_str());
      break;
    }
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
    if (connectFlag || deleteFlag)
      pub_node_list();
  }

  void RemoteControl::on_deleteNode_clicked(std_msgs::Header &cmd_remote)
  {
    std::string node_name = cmd_remote.frame_id;
    if (node_name == "")
    {
      ROS_WARN("deleteNode: node name empty");
      return;
    }

    for (auto &node_pair : nodeList)
    {
      auto &connect_nodes = node_pair.second.connect_node;
      for (auto it = connect_nodes.begin(); it != connect_nodes.end();)
      {
        if (*it == node_name)
        {
          it = connect_nodes.erase(it);
        }
        else
        {
          ++it;
        }
      }
    }
    auto node_it = nodeList.find(node_name);
    if (node_it != nodeList.end())
    {
      auto erase_begin = std::remove_if(line_lists.begin(), line_lists.end(),
                                        [&node_name](const std::pair<std::string, std::string> &line_pair)
                                        {
                                          return (line_pair.first == node_name) || (line_pair.second == node_name);
                                        });

      line_lists.erase(erase_begin, line_lists.end());
      nodeList.erase(node_it);
      pub_node_list();
      ROS_INFO("deleteNode: deleted node %s", node_name.c_str());
    }
    else
    {
      ROS_WARN("deleteNode: node %s not found in nodeList", node_name.c_str());
    }
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

  void RemoteControl::on_setGoal_clicked(std_msgs::Header &cmd_remote)
  {
    std::string goal_name = cmd_remote.frame_id;
    if (goal_name == "")
    {
      ROS_WARN("setGoal: goal name empty");
      return;
    }

    robot_task_msgs::robot_goal goal;
    goal.name = goal_name;
    goal.pose = nodeList[goal.name].pose;
    pubGoalSend.publish(goal);
  }

  void RemoteControl::on_insertNode_clicked(std_msgs::Header &cmd_remote)
  {
    std::string node_name = cmd_remote.frame_id;
    if (node_name == "")
    {
      ROS_WARN("insertNode: node name empty");
      return;
    }
    nodeList[node_name].pose = robotPose.pose;
    pub_node_list();
  }

  void RemoteControl::on_nodeClear_clicked(std_msgs::Header &cmd_remote)
  {
    nodeList.clear();
    line_lists.clear();
    pub_node_list();
  }

  void RemoteControl::on_importNode_clicked(std_msgs::Header &cmd_remote)
  {

    import_nodes(node_file);
  }

  void RemoteControl::import_nodes(std::string fileName)
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
      }
      ROS_INFO("node num: %ld", size);
      pub_node_list();
    }
  }

  void RemoteControl::on_saveNode_clicked(std_msgs::Header &cmd_remote)
  {

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
    ofs.open(node_file);
    assert(ofs.is_open());
    ofs << json_file;
    ofs.close();
    ROS_INFO("save nodes to %s", node_file.c_str());
  }

  void RemoteControl::on_stopNavigation_clicked(std_msgs::Header &cmd_remote)
  {
    robot_task_msgs::robot_navigation_cmd navigationStop;
    navigationStop.cmd_type = 0;
    pubNavigationStart.publish(navigationStop);
  }

  void RemoteControl::on_startNavigation_clicked(std_msgs::Header &cmd_remote)
  {
    robot_task_msgs::robot_navigation_cmd navigationStart;
    navigationStart.cmd_type = 1;
    pubNavigationStart.publish(navigationStart);
  }

  void RemoteControl::on_clearPath_clicked(std_msgs::Header &cmd_remote)
  {
    robot_task_msgs::robot_navigation_cmd navigationClear;
    navigationClear.cmd_type = 2;
    pubNavigationStart.publish(navigationClear);
  }

  void RemoteControl::on_cross_clicked(std_msgs::Header &cmd_remote)
  {
    cmd.task_type = 0x0D;
    cmd.cross.type = 1;
    pubRobotCmd.publish(cmd);
  }

  void RemoteControl::on_crossSpeedSet_clicked(std_msgs::Header &cmd_remote)
  {
    cmd.task_type = 0x0E;
    pubRobotCmd.publish(cmd);
  }

  void RemoteControl::on_startCharge_clicked(std_msgs::Header &cmd_remote)
  {
    robot_task_msgs::robot_navigation_cmd navigationStart;
    navigationStart.cmd_type = 3;
    pubNavigationStart.publish(navigationStart);
    ROS_INFO("on_startCharge_clicked: publish start charge cmd");
  }

  void RemoteControl::on_stopCharge_clicked(std_msgs::Header &cmd_remote)
  {
    robot_task_msgs::robot_navigation_cmd navigationStart;
    navigationStart.cmd_type = 4;
    pubNavigationStart.publish(navigationStart);
  }

  void RemoteControl::on_setCharge_clicked(std_msgs::Header &cmd_remote)
  {
    std::string name = cmd_remote.frame_id;
    if (name.empty())
    {
      ROS_WARN("setCharge: name empty");
      return;
    }
    for (std::map<std::string, robot_task_msgs::robot_goal>::iterator it = nodeList.begin(); it != nodeList.end(); ++it)
    {
      if (it->second.name == name)
        it->second.is_charge = !it->second.is_charge;
      else
        it->second.is_charge = false;
    }
    ROS_INFO("setCharge: %s is_charge=%d", name.c_str(), nodeList[name].is_charge);
    pub_node_list();
  }

  void RemoteControl::on_agvInit_clicked(std_msgs::Header &cmd_remote)
  {
    cmd.task_type = 0x04;
    pubRobotCmd.publish(cmd);
  }

  void RemoteControl::on_ConnectSet_clicked(std_msgs::Header &cmd_remote)
  {
    deleteFlag = false;
    firstDeleteFlag = false;

    connectFlag = !connectFlag;
    if (connectFlag)
      firstConnectFlag = true;

    ROS_INFO("on_ConnectSet_clicked: connectFlag=%d", connectFlag);
  }

  void RemoteControl::on_deleteSet_clicked(std_msgs::Header &cmd_remote)
  {
    connectFlag = false;
    firstConnectFlag = false;

    deleteFlag = !deleteFlag;
    if (deleteFlag)
      firstDeleteFlag = true;
    ROS_INFO("on_deleteSet_clicked: deleteFlag=%d", deleteFlag);
  }

  void RemoteControl::on_setDirection_clicked(std_msgs::Header &cmd_remote)
  {
    std::string name = cmd_remote.frame_id;
    if (name.empty())
    {
      ROS_WARN("setDirection: name empty");
      return;
    }
    nodeList[name].dir_type = nodeList[name].dir_type == 0 ? 1 : 0;
    pub_node_list();
  }
  void RemoteControl::on_debugLaser_clicked(std_msgs::Header &cmd_remote)
  {
    static bool debug_flag = false;
    debug_flag = !debug_flag;
    std_msgs::Int8 debug_cmd;
    debug_cmd.data = debug_flag ? 1 : 0;
    pubDebugLaser.publish(debug_cmd);
    ROS_INFO("on_debugLaser_clicked: debug_flag=%d", debug_flag);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "remote_control_node");
  ros::NodeHandle n;
  remote_control::RemoteControl node(n);
  ros::spin();
  return 0;
}