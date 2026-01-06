
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef PATH_PLAN_HPP_
#define PATH_PLAN_HPP_

#include <iostream>
#include <string>
#include "queue"
#include "sstream"
#include "set"
#include "string.h"
#include "math.h"
#include <functional>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d.h>
#include <robot_task_msgs/robot_goal.h>
#include <robot_task_msgs/robot_goal_list.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace navigation
{

#define INF 200000000

  class PathPlan
  {
  public:
    struct Node
    {
      int id;
      std::string name;
      std::vector<std::pair<int, float>> connect_node;
      bool isSet = false;
      bool isNavNode = false;
      Node(int id1, std::string n)
      {
        id = id1;
        name = n;
      }
    };

    std::vector<Node> nodeList;
    std::map<std::string, int> name_id;
    std::vector<std::string> short_path;
    float costs = 0;
    float minCost = INF;
    float minCost_rotate = INF;
    std::vector<std::string> short_path_all;
    float costs_all = 0;
    std::map<std::string, robot_task_msgs::robot_goal> node_list_source;
    int num_path = 6;
    float rotate_weght = 0; // 5/M_PI;
    float goal_rotate_weight = 50 / M_PI;
    std::string start_test;
    float **cost;
    unsigned int **previse_node;

  public:
    PathPlan()
    {
      cost = nullptr;
      previse_node = nullptr;
    }
    ~PathPlan()
    {
      if (cost != nullptr)
      {
        delete cost[0];
        delete[] cost;
      }
      if (previse_node != nullptr)
      {
        delete previse_node[0];
        delete[] previse_node;
      }
    }

    void ConvertToNode(std::map<std::string, robot_task_msgs::robot_goal> &list)
    {
      nodeList.clear();
      name_id.clear();
      short_path.clear();
      costs = 0;
      node_list_source = list;
      int id = 0;
      for (std::map<std::string, robot_task_msgs::robot_goal>::iterator it = node_list_source.begin(); it != node_list_source.end(); ++it, ++id)
      {
        Node node(id, it->first);
        name_id[it->first] = id;
        nodeList.push_back(node);
      }
      for (std::map<std::string, robot_task_msgs::robot_goal>::iterator it = node_list_source.begin(); it != node_list_source.end(); ++it, ++id)
      {
        for (int i = 0; i < it->second.connect_node.size(); i++)
        {
          float cost = hypot(it->second.pose.position.x - node_list_source[it->second.connect_node[i]].pose.position.x, it->second.pose.position.y - node_list_source[it->second.connect_node[i]].pose.position.y);
          nodeList[name_id[it->first]].connect_node.push_back(std::make_pair(name_id[it->second.connect_node[i]], cost));
        }
      }
      FloydWarshall();
    }

    void dfs(int start, int end, std::vector<std::string> &path)
    {
      if (start == end)
      {
        if (nodeList[start].isNavNode)
        {
          float rotate_cost = AddRotateCosts(path, nodeList[start].name);
          costs = costs + rotate_cost;
          if (costs <= minCost_rotate)
          {
            path.push_back(nodeList[start].name);
            short_path = path;
            path.pop_back();
            minCost_rotate = costs;
          }
          costs = costs - rotate_cost;
          return;
        }

        if (costs <= minCost)
        {
          path.push_back(nodeList[start].name);
          short_path = path;
          path.pop_back();
          minCost = costs;
        }
        return;
      }

      for (int i = 0; i < nodeList[start].connect_node.size(); i++)
      {
        if (nodeList[start].isSet == false)
        {
          costs += nodeList[start].connect_node[i].second;
          int rotate = 0;
          if (rotate_weght != 0 && path.size() > 2)
          {
            double angle_1 = atan2(node_list_source[path[path.size() - 1]].pose.position.y - node_list_source[path[path.size() - 2]].pose.position.y, node_list_source[path[path.size() - 1]].pose.position.x - node_list_source[path[path.size() - 2]].pose.position.x);
            double angle_2 = atan2(node_list_source[nodeList[start].name].pose.position.y - node_list_source[path[path.size() - 1]].pose.position.y, node_list_source[nodeList[start].name].pose.position.x - node_list_source[path[path.size() - 1]].pose.position.x);
            rotate = (fabs(angle_1 - angle_2) * rotate_weght);
          }
          costs += rotate;

          if (costs > minCost)
          {
            costs -= nodeList[start].connect_node[i].second;
            costs -= rotate;
            continue;
          }
          nodeList[start].isSet = true;
          path.push_back(nodeList[start].name);
          dfs(nodeList[start].connect_node[i].first, end, path);
          path.pop_back();
          nodeList[start].isSet = false;
          costs -= rotate;
          costs -= nodeList[start].connect_node[i].second;
        }
      }
    }

    void bfs(int start, int end, std::vector<std::string> &path)
    {
      std::map<int, std::pair<int, float>> nodes_map;
      std::queue<int> visit_node;
      nodes_map[start] = std::make_pair(INF, 0);
      visit_node.push(start);
      nodeList[start].isSet = true;
      while (!visit_node.empty())
      {
        start = visit_node.front();
        visit_node.pop();
        for (int i = 0; i < nodeList[start].connect_node.size(); i++)
        {
          if (!nodeList[nodeList[start].connect_node[i].first].isSet)
          {
            nodes_map[nodeList[start].connect_node[i].first] = std::make_pair(start, 0);
            visit_node.push(nodeList[start].connect_node[i].first);
            nodeList[nodeList[start].connect_node[i].first].isSet = true;
            if (nodeList[start].connect_node[i].first == end)
            {
              break;
            }
          }
        }
      }
      path.push_back(nodeList[end].name);
      int end_temp = end;
      while (nodes_map[end].first != INF)
      {
        path.push_back(nodeList[nodes_map[end].first].name);
        for (int i = 0; i < nodeList[end].connect_node.size(); i++)
        {
          if (nodeList[end].connect_node[i].first == nodes_map[end].first)
          {
            costs += nodeList[end].connect_node[i].second;
            break;
          }
        }
        end = nodes_map[end].first;
      }
      std::reverse(path.begin(), path.end());
      minCost_rotate = costs + AddRotateCosts(path, nodeList[end_temp].name);
      for (int i = 0; i < nodeList.size(); i++)
        nodeList[i].isSet = false;
      short_path = path;
    }

    float AddRotateCosts(std::vector<std::string> path, std::string name)
    {
      if (path.size() < 1)
        return 0;
      tf::Quaternion quat;
      tf::quaternionMsgToTF(node_list_source[name].pose.orientation, quat);
      double roll, pitch, yaw;
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      double angle = atan2(node_list_source[name].pose.position.y - node_list_source[path[path.size() - 1]].pose.position.y,
                           node_list_source[name].pose.position.x - node_list_source[path[path.size() - 1]].pose.position.x);
      return fabs(yaw - angle) * goal_rotate_weight;
    }

    std::string FindStartNode(geometry_msgs::PoseStamped &robotPose)
    {
      std::string currentNode;
      float min = INF;
      for (std::map<std::string, robot_task_msgs::robot_goal>::iterator it = node_list_source.begin(); it != node_list_source.end(); ++it)
      {
        float distance = hypot(it->second.pose.position.x - robotPose.pose.position.x, it->second.pose.position.y - robotPose.pose.position.y);
        if (distance < min)
        {
          min = distance;
          currentNode = it->first;
        }
      }
      return currentNode;
    }

    void GetFinalPath(std::vector<std::string> &path)
    {
      if (path.size() < 3)
        return;
      std::string firstNode = path[0];
      for (int i = 2; i < path.size(); i++)
      {
        double angle_1 = atan2(node_list_source[path[i - 1]].pose.position.y - node_list_source[firstNode].pose.position.y, node_list_source[path[i - 1]].pose.position.x - node_list_source[firstNode].pose.position.x);
        double angle_2 = atan2(node_list_source[path[i]].pose.position.y - node_list_source[path[i - 1]].pose.position.y, node_list_source[path[i]].pose.position.x - node_list_source[path[i - 1]].pose.position.x);
        /* if(fabs(angle_1 - angle_2) < 3*M_PI/180){
           path.erase(path.begin()+i-1);
           i=i-1;
         }else{*/
        firstNode = path[i - 1];
        //}
      }
    }

    std::vector<std::string> &FindPath(std::string start, std::string end)
    {
      short_path.clear();
      costs = 0;
      minCost = cost[name_id[start]][name_id[end]] + 0.5;
      minCost_rotate = INF;
      std::vector<std::string> path;
      if (node_list_source.empty())
        return path;

      //    bfs(name_id[start],name_id[end],path);
      //    costs=0;
      //    path.clear();
      dfs(name_id[start], name_id[end], path);
      GetFinalPath(short_path);

      return short_path;
    }

    static bool costs_sort(std::tuple<std::string, float> a, std::tuple<std::string, float> b)
    {
      return std::get<1>(a) < std::get<1>(b);
    }

    void CovertNodes(std::string start, std::vector<std::string> &nodes,
                     std::vector<std::vector<std::string>> &paths)
    {
      std::vector<std::tuple<std::string, float>> costs_to_start;
      for (int i = 0; i < nodes.size(); i++)
      {
        FindPath(start, nodes[i]);
        costs_to_start.push_back(std::tuple<std::string, float>(nodes[i], minCost_rotate));
      }
      std::sort(costs_to_start.begin(), costs_to_start.end(), costs_sort);
      std::vector<std::string> path;
      nodes.clear();
      int cout = 0;
      while (!costs_to_start.empty())
      {
        path.push_back(std::get<0>(costs_to_start[0]));
        costs_to_start.erase(costs_to_start.begin());
        if (cout == num_path - 1)
          break;
        cout++;
      }
      for (int i = 0; i < costs_to_start.size(); i++)
        nodes.push_back(std::get<0>(costs_to_start[i]));

      paths.push_back(path);
      if (nodes.empty())
        return;
      CovertNodes(path[num_path - 1], nodes, paths);
    }

    std::vector<std::string> FindPath(std::string start, std::vector<std::string> &nodes)
    {
      std::vector<std::vector<std::string>> paths;
      std::vector<std::string> all_path;
      float costs_all_temp = 0;
      if (nodeList.empty())
        return all_path;

      for (int i = 0; i < nodeList.size(); i++)
        nodeList[i].isNavNode = false;
      for (int i = 0; i < nodes.size(); i++)
      {
        nodeList[name_id[nodes[i]]].isNavNode = true;
      }

      //    CovertNodes(start,nodes,paths);
      //    for(int i=0;i<paths.size();i++){
      //      Find(start,paths[i]);
      //      costs_all_temp+=costs_all;
      //      for(int j=0;j<short_path_all.size()-1;j++)
      //        all_path.push_back(short_path_all[j]);
      //      start = short_path_all[short_path_all.size()-1];
      //    }
      //    all_path.push_back(short_path_all[short_path_all.size()-1]);

      //    while(!nodes.empty()){
      start = start_test;
      float cost_temp = INF;
      std::vector<std::string> path_temp;
      int count = 0;
      for (int i = 0; i < nodes.size(); i++)
      {
        //      FindPath(start,nodes[i]);
        //      if(minCost_rotate< cost_temp){
        //        cost_temp = minCost_rotate;
        //        path_temp = short_path;
        //        count=i;
        //      }
        if (cost[name_id[start]][name_id[nodes[i]]] < cost_temp)
        {
          cost_temp = cost[name_id[start]][name_id[nodes[i]]];
          // path_temp = short_path;
          count = i;
        }
      }
      path_temp = FindPath(start, nodes[count]);
      start_test = nodes[count];
      nodes.erase(nodes.begin() + count);
      for (int i = 0; i < path_temp.size(); i++)
        short_path_all.push_back(path_temp[i]);
      costs_all += cost_temp;
      if (!nodes.empty())
        short_path_all.pop_back();
      //    }

      //    short_path_all = all_path;
      //    costs_all = costs_all_temp;

      return short_path_all;
    }

    std::vector<std::string> Find(std::string start, std::vector<std::string> &nodes)
    {
      short_path_all.clear();
      float gobal_cost = INF;
      std::vector<std::string> gobal_short_path;
      std::string first_node = start;
      if (nodes.empty())
        return gobal_short_path;

      for (int i = 0; i < nodeList.size(); i++)
        nodeList[i].isNavNode = false;
      for (int i = 0; i < nodes.size(); i++)
      {
        nodeList[name_id[nodes[i]]].isNavNode = true;
      }

      std::map<std::string, std::map<std::string, std::pair<float, std::vector<std::string>>>> two_points_costs;
      std::sort(nodes.begin(), nodes.end());
      do
      {
        std::vector<std::string> nodes_temp = nodes;
        start = first_node;
        while (nodes_temp.size() > 0)
        {
          std::vector<std::string> path;
          std::map<std::string, std::map<std::string, std::pair<float, std::vector<std::string>>>>::iterator it = two_points_costs.find(start);
          if (it != two_points_costs.end())
          {
            std::map<std::string, std::pair<float, std::vector<std::string>>>::iterator it = two_points_costs[start].find(nodes_temp[0]);
            if (it != two_points_costs[start].end())
            {
              path = two_points_costs[start][nodes_temp[0]].second;
              minCost_rotate = two_points_costs[start][nodes_temp[0]].first;
            }
            else
            {
              path = FindPath(start, nodes_temp[0]);
              if (path.empty())
                return short_path_all;
              two_points_costs[start][nodes_temp[0]].second = path;
              two_points_costs[start][nodes_temp[0]].first = minCost_rotate;
            }
          }
          else
          {
            path = FindPath(start, nodes_temp[0]);
            if (path.empty())
              return short_path_all;
            two_points_costs[start][nodes_temp[0]].second = path;
            two_points_costs[start][nodes_temp[0]].first = minCost_rotate;
          }

          AddToPath(path, short_path_all);
          start = nodes_temp[0];
          nodes_temp.erase(nodes_temp.begin());
          costs_all += minCost_rotate;
          if (costs_all > gobal_cost)
            break;

          if (nodes_temp.empty())
          {
            short_path_all.push_back(start);
          }
        }

        if (costs_all < gobal_cost)
        {
          gobal_cost = costs_all;
          gobal_short_path = short_path_all;
        }
        costs_all = 0;
        minCost = INF;
        minCost_rotate = INF;
        short_path_all.clear();
      } while (std::next_permutation(nodes.begin(), nodes.end()));
      short_path_all = gobal_short_path;
      costs_all = gobal_cost;

      return short_path_all;
    }

    void InitMatrix(void)
    {
      if (cost)
      {
        delete cost[0];
        delete[] cost;
      }
      if (previse_node)
      {
        delete previse_node[0];
        delete[] previse_node;
      }

      int node_number = nodeList.size();

      cost = new float *[node_number];
      // 创建cost二维数组，并置原始数据
      cost[0] = new float[node_number * node_number];
      for (int i = 1; i < node_number; i++)
      {
        cost[i] = cost[i - 1] + node_number;
      }

      for (int i = 0; i < node_number; i++)
      {
        for (int j = 0; j < node_number; j++)
        {
          if (i != j)
            cost[i][j] = INF;
          else
            cost[i][j] = 0;
        }
      }
      // 创建previse_node二维数组，并置原始数据
      previse_node = new unsigned int *[node_number];
      previse_node[0] = new unsigned int[node_number * node_number];
      for (int i = 1; i < node_number; i++)
      {
        previse_node[i] = previse_node[i - 1] + node_number;
      }
      for (int i = 0; i < node_number; i++)
      {
        for (int j = 0; j < node_number; j++)
        {
          previse_node[i][j] = INF;
        }
      }

      // 初始化cost  previse_node
      for (int i = 0; i < nodeList.size(); i++)
      {
        int index_start = i;
        for (int j = 0; j < nodeList[i].connect_node.size(); j++)
        {
          int index_goal = nodeList[i].connect_node[j].first;
          cost[index_start][index_goal] = cost[index_goal][index_start] = nodeList[i].connect_node[j].second;
          previse_node[index_start][index_goal] = index_start;
          previse_node[index_goal][index_start] = index_goal;
        }
      }
    }

    void FloydWarshall(void)
    {
      InitMatrix();
      int node_number = nodeList.size();
      // PrintfMatrix(cost,node_number);
      // std::cout<<"************************************"<<std::endl;

      for (int n = 0; n < node_number; n++)
        for (int i = 0; i < node_number; i++)
          for (int j = 0; j < node_number; j++)
          {
            float cost_i_j = cost[i][j];
            float cost_i_n_j = cost[i][n] + cost[n][j];
            if (cost_i_j > cost_i_n_j)
            {
              cost[i][j] = cost_i_n_j;
              previse_node[i][j] = previse_node[n][j];
            }
          }
      // PrintfMatrix(previse_node,node_number);
    }

    template <typename T>
    void PrintfMatrix(T **matrix, int node_number)
    {
      for (int i = 0; i < node_number; i++)
      {
        for (int j = 0; j < node_number; j++)
        {
          std::cout << matrix[i][j] << " ";
        }
        std::cout << std::endl;
      }
    }

    void AddToPath(std::vector<std::string> &path1, std::vector<std::string> &path2)
    {
      for (int i = 0; i < path1.size() - 1; i++)
        path2.push_back(path1[i]);
    }

    void ConnectPathPoints(visualization_msgs::Marker &line_list, int type = 0)
    {
      std::vector<std::string> path = type == 0 ? short_path : short_path_all;
      line_list.header.frame_id = "map";
      line_list.header.stamp = ros::Time::now();
      line_list.ns = "shapes";
      line_list.action = visualization_msgs::Marker::ADD;
      line_list.id = 2;
      line_list.type = visualization_msgs::Marker::LINE_LIST;
      line_list.scale.x = 0.05;
      line_list.color.b = 25;
      line_list.color.g = 20;
      line_list.color.r = 0;
      line_list.color.a = 0.8;

      geometry_msgs::Point p;
      p.x = node_list_source[path[0]].pose.position.x;
      p.y = node_list_source[path[0]].pose.position.y;
      p.z = node_list_source[path[0]].pose.position.z;
      line_list.points.push_back(p);
      for (int i = 1; i < path.size(); i++)
      {
        geometry_msgs::Point p;
        p.x = node_list_source[path[i]].pose.position.x;
        p.y = node_list_source[path[i]].pose.position.y;
        p.z = node_list_source[path[i]].pose.position.z;
        line_list.points.push_back(p);
        line_list.points.push_back(p);
      }
      line_list.points.pop_back();
    }
  };

}
#endif
