#include "../include/laser_avoidance/laser_avoidance.hpp"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace laser_avoidance {

LaserAvoidance::LaserAvoidance(ros::NodeHandle nh, ros::NodeHandle pnh)
{
  nhandle = nh;
  phandle = pnh;

  pub_laser_filter = nhandle.advertise<sensor_msgs::LaserScan>("/laser_filter", 2);
  pub_avoidance_msg = nhandle.advertise<std_msgs::Int8>("/avoid_status", 2);
  pub_adjust_req = nhandle.advertise<std_msgs::Int8>("/adjust_req", 2);
  pub_adjust_motion = nhandle.advertise<robot_state_msgs::data_to_stm32>("/data_to_stm32", 2);
  pub_debug_msg = nhandle.advertise<std_msgs::String>("/debug_msg", 2);

  sub_avoidance_msg = nhandle.subscribe("/avoid_obstacle_control", 1, &LaserAvoidance::AvoidanceSetHandler, this);
  sub_laser_msgs = nhandle.subscribe("/scan", 5, &LaserAvoidance::LaserHandler, this);
  sub_adjust_cmd = nhandle.subscribe("/adjust_cmd", 5, &LaserAvoidance::AdjustSetHandler, this);
  sub_debug_cmd = nhandle.subscribe("/debug_cmd", 5, &LaserAvoidance::DebugSetHandler, this);

  LoadAvoidanceParam(workspace_path+"/install/share/laser_avoidance/config/laser_avoidance.yaml");

}

LaserAvoidance::~LaserAvoidance()
{

}

void LaserAvoidance::LoadAvoidanceParam(std::string dir)
{
  try
  {
      cv::FileStorage fs(dir.c_str(),cv::FileStorage::READ);
      if(!fs.isOpened()){
        std::cout<<"Cant't open param file!"<<std::endl;
        return;
      }
      
      float front_avoid_padding,behind_avoid_padding,left_avoid_padding,right_avoid_padding;

      fs["adjust_debug"]>>adjust_debug;
      fs["avoidance_enable"]>>avoidance_enable;
      fs["robot_width"]>>robot_width;
      fs["robot_length"]>>robot_length;
      fs["robot_center_to_front"]>>robot_center_to_front;
      fs["front_avoid_padding"]>>front_avoid_padding;
      fs["behind_avoid_padding"]>>behind_avoid_padding;
      fs["left_avoid_padding"]>>left_avoid_padding;
      fs["right_avoid_padding"]>>right_avoid_padding;
      fs["max_fitter"]>>max_fitter;
      fs["offset"]>>offset;
      fs["min_range"]>>min_range;
      fs["max_range"]>>max_range;
      std::cout<<"offset: "<<offset<<std::endl;

      robot_polygon.x_min = robot_center_to_front - robot_length;
      robot_polygon.x_max = robot_center_to_front;
      robot_polygon.y_min = -robot_width/2;
      robot_polygon.y_max = robot_width/2;

      move_stop_avoidance.x_min = robot_center_to_front;
      move_stop_avoidance.x_max = robot_center_to_front + front_avoid_padding;
      move_stop_avoidance.y_min = -robot_width/2-left_avoid_padding;
      move_stop_avoidance.y_max = robot_width/2+right_avoid_padding;

      move_dev_avoidance.x_min =  robot_center_to_front + front_avoid_padding;
      move_dev_avoidance.x_max = robot_center_to_front + front_avoid_padding+0.3;
      move_dev_avoidance.y_min = -robot_width/2-left_avoid_padding;
      move_dev_avoidance.y_max = robot_width/2+right_avoid_padding;

    //  float rad = hypot(robot_width/2,robot_center_to_front)+0.01;
    //  rotate_avoidance.x_min = 0;
    //  rotate_avoidance.x_max = rad;
    //  rotate_avoidance.y_min = -rad;
    //  rotate_avoidance.y_max = rad;

      rotate_avoidance.x_min = robot_center_to_front - robot_length;
      rotate_avoidance.x_max = robot_center_to_front;
      rotate_avoidance.y_min = -robot_width/2-left_avoid_padding;
      rotate_avoidance.y_max = robot_width/2+right_avoid_padding;

      fs.release();
      std::cout<<"load param ok!"<<std::endl;
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  
  
}

void LaserAvoidance::AvoidanceSetHandler(const std_msgs::Int8ConstPtr &msg)
{
      avoidance_enable = msg->data;
}

void LaserAvoidance::AdjustSetHandler(const std_msgs::Int8ConstPtr &msg)
{
      adjust_flag = msg->data;
}

void LaserAvoidance::DebugSetHandler(const std_msgs::Int8ConstPtr &msg)
{
      adjust_debug = msg->data;
}

void LaserAvoidance::LaserHandler(const sensor_msgs::LaserScanConstPtr &scan)
{

  sensor_msgs::PointCloud2 cloud;
  sensor_msgs::LaserScan laser;

  if(adjust_debug == 1 || adjust_flag ==1)
    LaserFilter(*scan);

  laser_resultion = scan->angle_increment;

  if(!tfListener.waitForTransform(scan->header.frame_id,"base_link",
                                  scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),ros::Duration(1))){
    std::cout<<"no tranfroms to base_link!"<<std::endl;
    return;
  }
  projector.transformLaserScanToPointCloud("base_link", *scan, cloud, tfListener);

  CloudToLaser(cloud,laser);

  std_msgs::Int8 avoid_status;
  if(avoidance_enable){
    AvoidanceDeal(cloud,avoid_status);
  }
  pub_avoidance_msg.publish(avoid_status);

}

void LaserAvoidance::AvoidanceDeal(const sensor_msgs::PointCloud2& cloud,
           std_msgs::Int8&  avoid_status)
{
  uintptr_t cp = (uintptr_t)cloud.data.data();
  for (uint32_t j = 0; j < cloud.height; ++j)
    for (uint32_t i = 0; i < cloud.width; ++i){
      float* fp= (float*)(cp +(i + j * cloud.width) * cloud.point_step);
      double x = fp[0];
      double y = fp[1];

    //************calculate avoid*********************//
      if(IsIntersects(move_dev_avoidance,x,y)){
        move_dev_avoidance.laser_count ++;
      }

      if(IsIntersects(move_stop_avoidance,x,y)){
        move_stop_avoidance.laser_count ++;
      }

      if(IsIntersects(rotate_avoidance,x,y) && !IsIntersects(robot_polygon,x,y)){
        rotate_avoidance.laser_count ++;
      }

    }
	//************send aviod msgs*********************//
  avoid_status.data |=((move_dev_avoidance.laser_count>10?1:0)<<0);
  avoid_status.data |=((move_stop_avoidance.laser_count>10?1:0)<<1);
  avoid_status.data |=((rotate_avoidance.laser_count>10?1:0)<<2);
  avoid_status.data |=(avoidance_enable<<3);
  move_dev_avoidance.laser_count=0;
  move_stop_avoidance.laser_count=0;
  rotate_avoidance.laser_count=0;
    
}

bool LaserAvoidance::IsIntersects(AvoidDirection& direction,float x,float y)
{
  if(x>direction.x_min&&x<direction.x_max&&
     y>direction.y_min&&y<direction.y_max){
    return true;
  }else
    return false;

}

void LaserAvoidance::CloudToLaser(const sensor_msgs::PointCloud2& msg,
                                  sensor_msgs::LaserScan &scan)
{
  static double last_time = ros::Time::now().toSec();

  if(ros::Time::now().toSec()-last_time>1.0){
    const size_t SIZE = 2.0 * M_PI / laser_resultion;
    scan.header = msg.header;
    scan.angle_increment = laser_resultion;
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.range_min = 0.0;
    scan.range_max = 30.0;
    scan.time_increment = 0.0;
    scan.ranges.resize(SIZE, INFINITY);
    scan.intensities.resize(SIZE);

    for (sensor_msgs::PointCloud2ConstIterator<float> it(msg, "x"); it != it.end(); ++it){
      const float x = it[0];  // x
      const float y = it[1];  // y
      const float i = it[3];    
      const int bin = (atan2f(y, x) + static_cast<float>(M_PI)) / laser_resultion;

      if ((bin >= 0) && (bin < static_cast<int>(SIZE))){
        scan.ranges[bin] = sqrtf(x * x + y * y);
        scan.intensities[bin] = i;
      }
    }

    last_time = ros::Time::now().toSec();
  }

}

void LaserAvoidance::LaserFilter(const sensor_msgs::LaserScan& msg)
{
    sensor_msgs::PointCloud2 cloud;
    sensor_msgs::LaserScan laser_msg = msg;
    int count =0;
    for(int i = 0; i < laser_msg.ranges.size(); i++){
      if(laser_msg.intensities[i] > max_fitter && (laser_msg.ranges[i]>min_range && laser_msg.ranges[i] <max_range)){
        count++;
      }else{
        //std::cout<< laser_msg.intensities[i]<<std::endl;
        laser_msg.ranges[i]=0;
      }
    }
    //std::cout<< count<<std::endl;
    pub_laser_filter.publish(laser_msg); 

    if(adjust_flag != 1 && adjust_debug!=1)
      return;

    if(!tfListener.waitForTransform(laser_msg.header.frame_id,"base_link",
                                    laser_msg.header.stamp + ros::Duration().fromSec(laser_msg.ranges.size()*laser_msg.time_increment),ros::Duration(1))){
      std::cout<<"no tranfroms to base_link!"<<std::endl;
      return;
    }
    projector.transformLaserScanToPointCloud("base_link", laser_msg, cloud, tfListener);

    float x_total = 0;
    float y_total=0;
    count =0;
    uintptr_t cp = (uintptr_t)cloud.data.data();
    for (uint32_t j = 0; j < cloud.height; ++j)
      for (uint32_t i = 0; i < cloud.width; ++i){
        float* fp= (float*)(cp +(i + j * cloud.width) * cloud.point_step);
        double x = fp[0];
        double y = fp[1];
      //************calculate label*********************//
        if(fabs(x)<0.2 && fabs(y) > 0.1){
          x_total+=x;
          y_total+=y;
          count++;
        }

      }
      if(count>2){
        x_total = x_total/count;
        y_total = y_total/count;
        x_total =x_total-offset;
        
        if(adjust_debug){
          std_msgs::String data;
	  if(fabs(x_total)<=0.003){
	        data.data = "yes,it's okay";
	  	std::cout<<"yes,it's okay"<<std::endl;
	  }else{
		std::cout<<"x: "<<x_total<<",please move x in 0.003"<<std::endl;
		data.data = "x: "+std::to_string(x_total)+",please move x in 0.003";
	  }
	  pub_debug_msg.publish(data);
	    return;
	}

        robot_state_msgs::data_to_stm32 cmd;
        cmd.task_type=2;
        cmd.running.type = 1;
        if(fabs(x_total)>0.003){
          cmd.running.speed= (x_total) >0?0.005:-0.005;
        }else{
          cmd.running.speed= 0;
          adjust_flag = false;
          std_msgs::Int8 req;
          req.data = 1;
          pub_adjust_req.publish(req);
          std::cout<<"adjust ok!"<<std::endl;
        }
        pub_adjust_motion.publish(cmd);

      }else{
        adjust_flag = false;
        std_msgs::Int8 req;
        req.data = 1;
        pub_adjust_req.publish(req);
        std::cout<<"no data received, adjust failed!"<<std::endl;
      }

}

}


