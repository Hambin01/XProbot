
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/navigation_plan/navigation_plan.hpp"
#include <costmap_2d/costmap_math.h>

namespace navigation
{

  Navigation::Navigation(ros::NodeHandle nh, ros::NodeHandle pnh) : navigation_client("go_to_node", true)
  {
    nhandle = nh;
    phandle = pnh;

    phandle.param("use_replay", use_replay, false);
    subGoal = nhandle.subscribe("/goal_node", 1, &Navigation::GoalSetCall, this);
    subTaskPoint = nhandle.subscribe("/navigation_set", 1, &Navigation::NavigationSetCall, this);
    subRobotPose = nhandle.subscribe("/robot_pose", 1, &Navigation::PoseCallBack, this);
    subRobotState = nhandle.subscribe("/robot_state", 1, &Navigation::StateCallBack, this);
    subNodeList = nhandle.subscribe("/goal_list", 1, &Navigation::NodeListCallBack, this);
    subTaskCmd = nhandle.subscribe("/robot_task_cmd", 1, &Navigation::TaskCmdCallBack, this);
    pubRobotCmd = nhandle.advertise<robot_state_msgs::data_to_stm32>("data_to_stm32", 2);
    pubCmdVel = nhandle.advertise<geometry_msgs::Twist>("cmd_vel", 2);
    pubTaskPoint = phandle.advertise<visualization_msgs::Marker>("task_point", 1);
    pubTaskReq = nhandle.advertise<robot_task_msgs::robot_task_req>("robot_task_req", 1);
    pubNavigationCancel = nhandle.advertise<actionlib_msgs::GoalID>("/go_to_node/cancel", 1);

    controlThread = new std::thread(&Navigation::MoveToGoal, this);
    taskReqTimer = phandle.createTimer(ros::Duration(0.2), &Navigation::TaskReq, this);

    while (!navigation_client.waitForServer(ros::Duration(5.0)))
    {
      LOG_IF(INFO, 20) << "Waiting for the go_to_node action server to come up";
      ros::Rate(10).sleep();
    }
    InitParam();
  }

  Navigation::~Navigation()
  {
    if (controlThread->joinable())
    {
      controlThread->join();
      delete controlThread;
      controlThread = nullptr;
    }
  }

  geometry_msgs::Pose Navigation::convert_pose_to_json(Json::Value &list)
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

  void Navigation::import_nodes(std::string fileName)
  {
    if (fileName == "")
    {
      LOG(ERROR) << "filename:" << fileName << "is error!";
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
        LOG(ERROR) << "filename parse  error!";
        return;
      }
      int size = list.size();

      nodeList.clear();
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
        }
        nodeList[name].putter_hight = list[i]["putter_hight"].asUInt();
      }
    }
  }

  void Navigation::InitParam(void)
  {
    import_nodes(workspace_path + "/install/nodes/my.json");
    chargePoint = "none";
    for (std::map<std::string, robot_task_msgs::robot_goal>::iterator it = nodeList.begin(); it != nodeList.end(); ++it)
    {
      if (it->second.is_charge)
        chargePoint = it->first;
    }
    patPlan.ConvertToNode(nodeList);
  }

  void Navigation::StateCallBack(const robot_state_msgs::robot_stateConstPtr data)
  {
    static int last_sensor_data=0;
    robotState = *data;
    //LOG(INFO) << "cross_state:"<<(int)robotState.current_task;
    if(last_sensor_data!=robotState.battery_voltage&&robotState.battery_voltage==0x00
    &&is_load_cargo) {
      have_cargo=true;
      LOG(INFO) << "cross have cargo!";
      }
    else if(!is_load_cargo) have_cargo=false;
    last_sensor_data=robotState.battery_voltage;
  }

  void Navigation::NodeListCallBack(const robot_task_msgs::robot_goal_listConstPtr list)
  {
    nodeList.clear();
    chargePoint = "none";
    for (int i = 0; i < list->node.size(); i++)
    {
      nodeList[list->node[i].name] = list->node[i];
      if (nodeList[list->node[i].name].is_charge)
        chargePoint = list->node[i].name;
    }

    patPlan.ConvertToNode(nodeList);
  }

  void Navigation::TaskCmdCallBack(const robot_task_msgs::robot_task_reqConstPtr msg)
  {
    robot_task_msgs::robot_task_req stateReq = *msg;

    int type = (int)msg->type;
    if (type != 4 && type != 8)
      taskReq = *msg;

    switch (type)
    {
    case MOVE_SET:
    {
      std::string name = std::to_string((int)msg->data);
      std::map<std::string, robot_task_msgs::robot_goal>::iterator it = nodeList.find(name);
      if (it != nodeList.end())
      {
        GoalSetCall(nodeList[name]);
      }
      action_type = MOVE;
    }
    break;

    case CROSS_SET_UP:
    case CROSS_SET_DOWN:
    {
      task_start_time = ros::Time::now();
      cross_flag = true;
      have_cargo=false;
      robot_state_msgs::data_to_stm32 cmd;
      int count = 0;
      int max_try_num=90;
      stateReq.type=msg->type;
      stateReq.data=msg->data;
      pubTaskReq.publish(taskReq);
      int data=msg->data;
      while(count <max_try_num && robotState.current_task!=RobotCrossing){
         cmd.task_type = 0x0D;
         cmd.cross.type =data;
         ros::Rate(15).sleep();
         pubRobotCmd.publish(cmd);
         LOG(WARNING) << "robot corss running failed! current_task:"<<(int)robotState.current_task;
         count++;
      }
      last_cross_type =  msg->data; 
      if(count ==max_try_num){
         LOG(WARNING) << "robot corss timeout!"<<cmd.cross.type;
         is_load_cargo=false;
      }else{ 
         //pwq
         if(msg->data<=2) is_load_cargo=true;
         else is_load_cargo=false; 
         LOG(INFO) << "robot corss cmd send...."<<cmd.cross.type<<";"<<is_load_cargo<<";"<<last_cross_type;
      }
         
    }
    break;
    case STATE_GET:
      stateReq.data = robotState.battery_voltage;
      stateReq.pose_x = robotPose.pose.position.x * 1000;
      stateReq.pose_y = robotPose.pose.position.y * 1000;
      stateReq.putter_height = robotState.putter_height;
      pubTaskReq.publish(stateReq);
      break;

    case PUTTER_SET:
    {
      robot_state_msgs::data_to_stm32 cmd;
      cmd.task_type = 0x0E;
      cmd.putter.hight = msg->putter_height;
      pubRobotCmd.publish(cmd);
      LOG(INFO) << "robot putter cmd send....";
    }
    break;

    case STOP_SET:
    {
      pubNavigationCancel.publish(cancel_goal);
      action_type = STOP;
      taskPoint.clear();
      LOG(INFO) << "robot task clear ok,stop now!";
    }
    break;

    case CHARGE_SET:
    {
      if (chargePoint == "none")
      {
        LOG(WARNING) << "charge point not set!";
      }
      pubNavigationCancel.publish(cancel_goal);
      GoalSetCall(nodeList[chargePoint]);
      if (taskPoint.empty())
      {
        action_type = STOP;
        break;
      }
      action_type = CHARGE_ACTION;
      LOG(INFO) << "robot go to charge!";
    }
    break;

    case ROBOT_YAW_GET:
    {
      tf::Quaternion quat;
      tf::quaternionMsgToTF(robotPose.pose.orientation, quat);
      double roll, pitch, yaw;
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      taskReq.pose_x = yaw * 1000;
      pubTaskReq.publish(taskReq);
    }
    break;

    case LAST_NODE_GET:
    {
      taskReq.data = now_node;
      pubTaskReq.publish(taskReq);
    }
    break;

    case CROSS_STATE:
    {
      static int count=3;
      if (cross_flag) {
        if(robotState.current_task == RobotCrossing){
          taskReq.data = 1;
          if ((ros::Time::now() - task_start_time).toSec() > 23){
            taskReq.data = 3;
            if(count<0)
            cross_flag = false;
            count--;
            LOG(INFO) << "cross timeout!";
          }
        }else{
           taskReq.data = 2; 
           if(count<0)
           cross_flag = false;
           count--;
           LOG(INFO) << "cross get cargo!";
        }
      }else{
        count=3;
        taskReq.data = 0;
      }
      pubTaskReq.publish(taskReq);
    }
    break;

    case TASK_STATE_GET:
    // {
    //   taskReq.data = task_state;
    //   pubTaskReq.publish(taskReq);
    // }
    {
      stateReq.data = last_node;
      stateReq.pose_x = task_state;
      stateReq.pose_y = robotPose.pose.position.y * 1000;
      if(robotState.odom_angle == 1)
        stateReq.putter_height = stateReq.putter_height | 0x01;

      if(task_state == 2)
        stateReq.putter_height = stateReq.putter_height | 0x10;
      pubTaskReq.publish(stateReq);
    }
    break;

    case ROBOT_GET:
    {
      //stateReq.data = last_node;
      stateReq.data=goal_node;
      stateReq.pose_x = task_state;
      if(robotState.current_task==RobotCrossing)stateReq.pose_y=last_cross_type;
      else stateReq.pose_y=0;
      if(robotState.odom_angle == 1)
        stateReq.putter_height = stateReq.putter_height | 0x01;

      if(task_state == 2)
        stateReq.putter_height = stateReq.putter_height | 0x10;

      pubTaskReq.publish(stateReq);
    }
    break;

    case TASK_STATE_GET2:
    {
      taskReq.data = task_state;
      pubTaskReq.publish(taskReq);
    }
    break;
    //pwq
    case CROSS_STATE2:
    {
      /*
      if(robotState.current_task == RobotCrossing) {
        LOG(INFO) << "cross start unload cargo!";
        have_cargo=false;
      }*/
      if(have_cargo&&robotState.current_task != RobotCrossing) taskReq.putter_height=0X01;
      else taskReq.putter_height=0X00;
      pubTaskReq.publish(taskReq);
    }
    break;
    default:
      break;
    }
  }

  //goals set
  void Navigation::GoalsSetCall(void)
  {
    patPlan.rotate_weght = 0;
    std::vector<std::string> nodes = {"142", "443", "433", "440", "439", "599", "447", "477"};
    //std::vector<std::string> nodes={"35","36","42","43","44","46","48","38"};
    //std::vector<std::string> nodes={"996","991","996","996","996","1046","1047","991","996","1046","1047","991","1047","1047","1043","1043","1043","1046","1045","1044","1043","988","988","996","988","988","1046","1046","1046","1046","1046","1046","988","988","988","1046","1046","1046","988","988","988","1046","1046","1046","988","988","988","883","883","883","876","883","883","883","1050","1051","1051","883","883","883","876","876","876","881","881","881","876","876","876","878","878","876","876","876","1001","996","1000","1043","996","1043","996","996","996","996","996","996","999","999","999","1043","1043","1043","999","999","1000","1043","1043","984","1043","1043","988","985","984","984","984","984","984","1043","1043","1043","1043","1043","1043","984","984","984","1043","1043","1043","985","985","985","1043","1043","1043","985","985","985","884","884","884","884","884","884","884","884","888","884","884","884","889","889","890","879","879","879","881","881","881","879","879","879","879","882","873","873","871","981","999","1003","981","986","981","999","999","981","999","999","999","1001","1002","1003","981","981","981","1000","1000","1000","982","981","981","986","986","984","983","981","981","986","986","984","981","981","981","986","986","984","981","981","981","981","981","981","981","981","981","889","889","889","981","981","981","889","889","889","889","889","889","889","891","891","889","889","889","892","892","894","870","870","868","874","874","868","868","868","868","874","872","872","870","870","870","1010","1003","1007","978","978","978","1003","1003","1003","1003","1003","1003","1006","1006","1007","978","978","978","1003","1003","1003","982","978","978","982","982","981","980","977","977","983","983","983","977","977","977","983","983","983","977","977","977","977","977","977","983","983","983","983","983","983","977","977","977","891","891","892","891","891","892","891","891","895","891","891","892","896","898","898","865","865","865","871","871","869","865","865","865","871","869","869","865","865","865","1007","1007","1011","976","979","979","1007","1007","1007","1007","1007","1007","1010","1010","1011","978","976","974","1007","1007","1007","977","976","976","979","980","976","977","976","974","1007","1010","1011","976","976","976","979","979","979","976","976","974","979","979","976","974","974","974","979","979","979","976","974","974","897","895","895","897","895","896","897","903","903","897","896","896","906","906","906","897","908","907","907","907","907","907","865","865","865","907","907","907","1011","1011","1015","976","976","971","976","1011","1011","1011","1011","1011","1014","1014","1015","976","976","976","1011","1011","1011","976","976","976","976","976","974","970","970","970","1011","1014","1015","976","976","976","976","976","976","970","970","970","976","976","976","971","971","971","976","976","976","970","970","970","906","906","906","905","906","906","905","906","906","905","906","906","915","917","916","907","907","907","907","907","907","907","907","907","915","860","860","1014","1014","1019","968","968","967","1014","1014","1014","1014","1014","1014","1018","1018","1019","972","967","967","1015","1014","1015","970","967","967","973","973","971","970","967","967","1015","1018","1019","967","967","967","973","973","973","967","967","967","967","967","967","967","967","967","973","973","973","967","967","967","914","914","914","914","914","914","914","914","917","914","914","914","918","920","920","914","914","917","914","914","917","914","914","916","920","921","921","1018","1018","1023","968","968","1018","968","1018","1018","1018","1018","1018","1022","1022","1023","969","969","969","1018","1018","1019","969","970","969","969","970","967","966","964","1023","1018","1020","1023","969","969","969","969","969","969","964","964","964","969","969","969","964","964","964","969","969","969","964","964","964","918","918","918","918","918","918","918","918","936","918","918","918","935","935","936","921","921","921","921","921","921","921","921","921","935","935","936","1022","1022","1027","1022","1022","1022","1022","1022","1022","1022","1022","1022","1025","1026","1027","1022","1022","1022","1022","1022","1023","963","966","964","1023","1023","1025","1025","1026","1027","1022","1026","1025","957","957","957","966","966","966","958","958","958","957","957","957","958","958","958","966","966","966","958","958","958","934","934","934","934","934","934","934","934","934","934","934","934","940","940","940","934","934","934","934","934","934","934","934","934","946","943","943","1026","1026","1032","1026","1026","1026","1026","1026","1026","1026","1026","1026","1030","1031","1032","1026","1026","1026","1027","1026","1028","958","958","962","1027","1028","1029","1029","960","962","1027","1028","1029","958","958","958","958","958","958","1037","1037","1037","958","958","958","1037","1037","958","963","958","958","1037","1037","1037","939","939","939","939","939","939","939","939","956","939","939","939","955","955","956","939","939","939","939","939","939","939","939","939","956","955","956","853","855","853","853","1082","1082","1082","853","855","853","857","856","853","1066","1066","1066","857","855","853","1064","1064","1064","1064","1064","1064","853","853","853","874","874","874","874","874","877","877","880","879","874","857","855","855","857","855","855","877","877","877","878","878","878","878","878","878","853","853","853","842","842","845","842","842","842","845","845","845","842","842","842","842","842","842","842","842","842","846","846","845","842","842","842","846","846","846","842","842","842","846","846","842","842","842","866","866","866","899","866","899","899","866","1091","1091","1091","1091","1091","1091","1096","1096","1096","1091","1091","1091","846","845","844","844","844","844","849","849","846","844","844","845","845","844","844","844","844","849","849","849","845","845","844","844","844","844","844","844","844","844","844","845","850","845","849","845","845","850","845","845","849","847","850","847","845","845","873","868","868","868","868","868","873","871","871","868","1086","1086","1086","1086","1086","1086","1086","1086","1086","1091","1091","1091","1086","1086","1086","848","848","848","848","848","848","854","853","853","850","848","848","848","847","846","846","846","846","854","854","854","847","847","846","846","846","846","848","848","846","874","873","873","847","847","846","848","848","848","848","848","848","848","848","848","848","848","872","872","872","872","853","853","853","853","853"};
    //std::string nodes="544";
    //   for(std::map<std::string,robot_task_msgs::robot_goal>::iterator it=nodeList.begin(); it!=nodeList.end(); ++it){
    //    nodes.push_back(it->first);
    //  }
    std::sort(nodes.begin(), nodes.end());
    nodes.erase(std::unique(nodes.begin(), nodes.end()), nodes.end());
    std::string start = patPlan.FindStartNode(robotPose);
    patPlan.start_test = start;
    ros::Time current_time1 = ros::Time::now();
    taskPoint = patPlan.FindPath(start, nodes);
    while (!nodes.empty())
    {

      taskPoint = patPlan.FindPath(start, nodes);

      if (taskPoint.size() < 1)
      {
        LOG(WARNING) << "No Path to This node!";
        return;
      }
      visualization_msgs::Marker nodePoint;
      patPlan.ConnectPathPoints(nodePoint, 1);
      pubTaskPoint.publish(nodePoint);
      ros::Rate(1).sleep();
    }
    for (int i = 0; i < patPlan.short_path_all.size(); i++)
      std::cout << patPlan.short_path_all[i] << "->";
    std::cout << patPlan.costs_all << "<-" << std::endl;
    std::cout << "time:" << (ros::Time::now() - current_time1).toSec() << std::endl;
  }

  //one goal set
  void Navigation::GoalSetCall(robot_task_msgs::robot_goal goal)
  {
    boost::unique_lock<boost::shared_mutex> lockDeal(mutexControl);

    if (action_type != STOP)
    {
      LOG(WARNING) << "please stop current task!";
      return;
    }
    taskPoint.clear();
    patPlan.rotate_weght = 0;
    goalNode = goal;
    std::string name = goal.name;
    std::map<std::string, robot_task_msgs::robot_goal>::iterator it = nodeList.find(name);
    if (it != nodeList.end())
    {
      ros::Time current_time1 = ros::Time::now();

      std::string start = patPlan.FindStartNode(robotPose);
      taskPoint = patPlan.FindPath(start, name);
      for (int i = 0; i < patPlan.short_path.size(); i++)
        std::cout << patPlan.short_path[i] << "->";
      std::cout << "time:" << (ros::Time::now() - current_time1).toSec() << std::endl;
      if (taskPoint.size() < 1)
      {
        LOG(WARNING) << "No Path to This node!";
        return;
      }
      visualization_msgs::Marker nodePoint;
      patPlan.ConnectPathPoints(nodePoint, 0);
      pubTaskPoint.publish(nodePoint);
    }
    if (taskPoint.size() < 1)
    {
      LOG(WARNING) << "No Path to This node!";
    }
    goal_node = std::atoi(goal.name.c_str());
  }

  void Navigation::NavigationSetCall(robot_task_msgs::robot_navigation_cmd cmd)
  {
    switch (cmd.cmd_type)
    {
    case 0:
      pubNavigationCancel.publish(cancel_goal);
      action_type = STOP;
      break;

    case 1:
      if (action_type == STOP)
      {
        action_type = MOVE;
      }
      break;
    case 2:
    {
      pubNavigationCancel.publish(cancel_goal);
      action_type = STOP;
      taskPoint.clear();
      visualization_msgs::Marker nodePoint;
      pubTaskPoint.publish(nodePoint);
      break;
    }
    case 3:
      if (chargePoint == "none")
      {
        LOG(WARNING) << "charge point not set!";
      }
      pubNavigationCancel.publish(cancel_goal);
      GoalSetCall(nodeList[chargePoint]);
      if (taskPoint.empty())
      {
        LOG(INFO) << "no path to charge!";
        return;
      }
      action_type = CHARGE_ACTION;
      LOG(INFO) << "robot go to charge!";
      break;

    case 4:
      if (robotState.current_task != RobotCharging)
      {
        LOG(WARNING) << "robot not charge!";
        break;
      }
      GoalSetCall(nodeList[chargePoint]);
      action_type = MOVE;
      break;

    default:
      break;
    }
  }

  void Navigation::PoseCallBack(geometry_msgs::PoseStampedConstPtr pose)
  {
    boost::unique_lock<boost::shared_mutex> lockDeal(mutexControl);
    robotPose = *pose;
  }

  void Navigation::TaskReq(const ros::TimerEvent &event)
  {
    if (taskReq.type == 0 || action_type == STOP)
      return;

    switch (taskReq.type)
    {
    case 1:
      pubTaskReq.publish(taskReq);
      if (taskReq.data == 2)
      {
        taskReq.type = 0;
        taskReq.data = 0;
      }
      break;
    case 2:
    case 3:
      taskReq.data = robotState.crossSensorValue == 1 ? 2 : 1;
      pubTaskReq.publish(taskReq);
      if (taskReq.data == 2)
      {
        taskReq.type = 0;
        taskReq.data = 0;
      }
      break;

    default:
      break;
    }
  }

  void Navigation::SetSpeed(float speed, float radius, float angle)
  {
    cmd.task_type = 2;
    cmd.running.type = 1;
    cmd.running.angle = angle;
    cmd.running.radius = radius;
    cmd.running.speed = speed;
    pubRobotCmd.publish(cmd);
    geometry_msgs::Twist  vel;
    vel.linear.x = speed;
    vel.angular.z = radius;
    pubCmdVel.publish(cmd);
  }

  void Navigation::MoveToGoal(void)
  {
    while (ros::ok())
    {
      if (action_type != STOP)
      {

        switch (action_type)
        {
        case MOVE:
          if (!StopChargeAction())
            break;

          if (MoveAction())
          {
            action_type = PUTTER_ACTION;
          }
          else
            action_type = RUN_ERROR;
          break;

        case PUTTER_ACTION:
          if (TaskAction())
          {
            action_type = STOP;
          }
          break;

        case RUN_ERROR:
          pubNavigationCancel.publish(cancel_goal);
          action_type = STOP;
          taskPoint.clear();
          break;

        case CHARGE_ACTION:
          if (robotState.current_task == RobotCharging){
            action_type = STOP;
            break;
          }

          if (MoveAction()){
            ChargeAction();
            action_type = STOP;
          }
          else
            action_type = RUN_ERROR;
          break;

        default:
          break;
        }
      }
      ros::Duration(0.05).sleep();
    }
  }

  bool Navigation::StopChargeAction(void)
  {
    if (robotState.current_task == RobotCharging || back_flag)
    {
      int count = 0;
      cmd.task_type = 0x04;
      pubRobotCmd.publish(cmd);
      LOG(INFO) << "send init msgs!";
      while (count < 10 * 5 && robotState.current_task != RobotIdle)
      {
        count++;
        ros::Rate(5).sleep();
      }
      if (robotState.current_task != RobotIdle)
        LOG(WARNING) << "stop charge robot init failed!";

      count = 0;
      while (count < 3 * 5 && action_type != STOP)
      {
        count++;
        SetSpeed(0.04, 0, 0);
        ros::Rate(5).sleep();
      }
      SetSpeed(0, 0, 0);
      back_flag = false;
    }
    return true;
  }

  bool Navigation::ChargeAction(void)
  {
    if (robotState.current_task != RobotCharging)
    {
      int count = 0;
      while (count < 40 * 5 && action_type != STOP)
      {
        count++;
        SetSpeed(-0.02, 0, 0);
        ros::Rate(5).sleep();
        if (robotState.current_task == RobotCharging)
        {
          LOG(INFO) << "charge OK!";
          return true;
        }
      }
      SetSpeed(0, 0, 0);
      back_flag = true;
      LOG(WARNING) << "charge timeout!";
      return false;
    }
    return true;
  }

  bool Navigation::MoveAction(void)
  {
    std::string cur_node = patPlan.FindStartNode(robotPose);
    last_node = now_node;
    now_node = 0;

    for (int i = 0; taskPoint.size(); i++)
    {
      motion_control::robotNavGoal goal;
      goal.start_pose.pose = robotPose.pose;
      goal.goal_pose.pose = nodeList[taskPoint[0]].pose;
      goal.dir_type = (cur_node==nodeList[taskPoint[0]].name)?0:nodeList[cur_node].dir_type;
      if (taskPoint.size() > 1)
        goal.goal_type = 1;
      navigation_client.sendGoal(goal);
      task_state = 2;
      LOG(INFO) << "Send the target goal:" << nodeList[taskPoint[0]].name;
      float navigation_duration_time = hypot(goal.start_pose.pose.position.x - goal.goal_pose.pose.position.x, goal.start_pose.pose.position.y - goal.goal_pose.pose.position.y) / 0.1 +
                                       fabs(goal.start_pose.pose.position.z - goal.goal_pose.pose.position.z) / 0.1 + 100;
      navigation_client.waitForResult(ros::Duration(navigation_duration_time));
      if ((navigation_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) || (navigation_client.getState() == actionlib::SimpleClientGoalState::RECALLED))
      {
        LOG(INFO) << "navigation get state: " << navigation_client.getState().toString();
        last_node = std::atoi(nodeList[taskPoint[0]].name.c_str());
        cur_node = nodeList[taskPoint[0]].name;
        taskPoint.erase(taskPoint.begin());
      }
      else
      {
        LOG(ERROR) << "navigation get state: " << navigation_client.getState().toString();
        now_node = last_node;
        task_state = 5;
        return false;
      }
    }
    now_node = goal_node;
    last_node  = now_node;
    task_state = 4;
    return true;
  }

  bool Navigation::TaskAction(void)
  {
    static bool crossFlag = false;

    /*  if(abs(goalNode.putter_hight - robotState.putter_height)>2){
    cmd.task_type = 0x0E;
    cmd.putter.hight = goalNode.putter_hight;
    return false;
  }*/

    if (goalNode.cross_type != 0 && !crossFlag)
    {
      cmd.task_type = 0x0D;
      cmd.cross.type = goalNode.cross_type;
      sleep(1);
      crossFlag = true;
      return false;
    }

    if (robotState.current_task == 0)
    {
      crossFlag = false;
      return true;
    }
    return true;
  }

}