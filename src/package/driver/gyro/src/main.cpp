#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <serial/serial.h>
#include <vector>
#include <math.h>
#include <time.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <ros/console.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>


using namespace std;

double roll,pitch,yaw;
double accx,accy,accz;
double gyrox,gyroy,gyroz;


int address=0x01;
int newaddress;
/**
 * @brief setCommand
 * 0:无设置命令
 * 1:设置频率
 * 2:设置波特率
 * 3:设置地址
 * 4:复位方位角
 * 5:复位角速度
 * 6:设置输出类型
 */
int setCommand=0;
/**
 * @brief kind
 * 0:９轴输出
 * 1:标准1
 * 2:标准2
 * 3:标准3
 */
int kind=0;//
int newkind;
serial::Serial ser;
ros::Publisher imu_pub;
ros::Subscriber imu_pose_init_sub;
ros::Publisher odom_pub;

typedef boost::unique_lock<boost::mutex> write_Lock;
boost::mutex mutex1;

//********para of imu*******
Eigen::Vector3d tmp_P=Eigen::Vector3d(0, 0, 0); //t
Eigen::Quaterniond tmp_Q=Eigen::Quaterniond::Identity();//R
Eigen::Vector3d tmp_V=Eigen::Vector3d(0, 0, 0);

Eigen::Vector3d acc_0=Eigen::Vector3d(0, 0, 0);
Eigen::Vector3d gyr_0=Eigen::Vector3d(0, 0, 0);

Eigen::Vector3d tmp_Ba=Eigen::Vector3d(0, 0, 0);
Eigen::Vector3d tmp_Bg=Eigen::Vector3d(0, 0, 0);

double g_last_imu_time = -1.0;

double imu_angular_vel_rz=0.0;
double const_imu_linear_acc_x = 0.0;
double const_imu_linear_acc_y =0.0;
Eigen::Vector3d imu_angular_vel;
Eigen::Vector3d imu_linear_acc;
clock_t current_time,init_time;

Eigen::Vector3d g = Eigen::Vector3d(0,0,9.806649999788);

clock_t latest_time;

int first = 1;

using namespace std;

template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
    {
        typedef typename Derived::Scalar Scalar_t;
 
        Eigen::Quaternion<Scalar_t> dq;
        Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
        half_theta /= static_cast<Scalar_t>(2.0);
        dq.w() = static_cast<Scalar_t>(1.0);
        dq.x() = half_theta.x();
        dq.y() = half_theta.y();
        dq.z() = half_theta.z();
        return dq;
    }


void predict(Eigen::Vector3d imu_linear_acc, Eigen::Vector3d&  imu_angular_vel,clock_t t)
{
    //std::cout<<imu_angular_vel(2)<<std::endl;

    double dt =(double) (t - g_last_imu_time)/CLOCKS_PER_SEC;

    g_last_imu_time = t;

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + imu_angular_vel) - tmp_Bg;

    tmp_Q = tmp_Q * deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (imu_linear_acc - tmp_Ba) - g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = imu_linear_acc;
    gyr_0 = imu_angular_vel;

    

/*

    double dt =(double) (t - g_last_imu_time)/CLOCKS_PER_SEC;
    g_last_imu_time = t;
    tmp_V+=tmp_Q*(imu_linear_acc-g)*dt;

    Eigen::Vector3d acc=  tmp_Q*(imu_linear_acc-g);
    tmp_P = tmp_P + tmp_Q* tmp_V*dt+0.5*dt*dt*acc ;
    tmp_Q = tmp_Q * Eigen::Quaterniond(1, 0 , 0, 0.5*imu_angular_vel(2)*dt);
*/
    //cout<<"tmp_Q eular"<<(180/M_PI)*tmp_Q.matrix().eulerAngles(0,1,2)<<endl;

	static tf2_ros::TransformBroadcaster br;
  	geometry_msgs::TransformStamped transformStamped;
  
  	transformStamped.header.stamp = ros::Time::now();
  	transformStamped.header.frame_id = "world";
  	transformStamped.child_frame_id = "imu_odometry";
  	transformStamped.transform.translation.x = tmp_P(0);
  	transformStamped.transform.translation.y = tmp_P(1);
  	transformStamped.transform.translation.z = tmp_P(2);

  	transformStamped.transform.rotation.x = tmp_Q.x();
  	transformStamped.transform.rotation.y = tmp_Q.y();
  	transformStamped.transform.rotation.z = tmp_Q.z();
  	transformStamped.transform.rotation.w = tmp_Q.w();

  	br.sendTransform(transformStamped);

 
    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "world";
 
    //set the position
    odom.pose.pose.position.x = tmp_P(0);
    odom.pose.pose.position.y = tmp_P(1);
    odom.pose.pose.position.z = tmp_P(2);

    odom.pose.pose.orientation.x = tmp_Q.x();
    odom.pose.pose.orientation.y = tmp_Q.y();
    odom.pose.pose.orientation.z = tmp_Q.z();
    odom.pose.pose.orientation.w = tmp_Q.w();
 
    //set the velocity
    odom.child_frame_id = "imu_odometry";
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = 0;
 
    //publish the message
    odom_pub.publish(odom);

    //std::cout<<" dt: " <<  dt <<std:: cout<<"x="<<tmp_P(0)<<" y="<<tmp_P(1)<<" z="<<tmp_P(2)<<std::endl;


}


void setFrequency(int set=0x03)
{
    unsigned char buf[6];
    buf[0]=0x68;
    buf[1]=0x05;
    buf[2]=address;
    buf[3]=0x0C;
    buf[4]=set;

    unsigned char cnt = 0;
    for(int i=1;i<5;i++)
    {
        cnt +=(buf[i]);
    }

    buf[5] = cnt;
    setCommand=1;
    write_Lock lck(mutex1);
    ser.write(buf,6);
    ROS_INFO("setFrequency = %d", set);
}

int setFrequencyReturn(std::vector<unsigned char> buf,int size=6)
{
    ROS_INFO("setFrequencyReturn ok");
    int checksum=0;
    for(int i=1;i<size-1;i++)
    {
        checksum=(checksum+buf[i])&0xFF;
    }
    if((buf[0]==0x68)&&(buf[1]==0x05)&&(buf[2]==address)&&(buf[3]==0x8C)&&(buf[5]==checksum))
    {
        if(buf[4]==0x00)
        {
            ROS_INFO("setFrequencyReturn 1");
            setCommand=0;
            return 1;
        }
        else if(buf[4]==0xFF)
        {
            ROS_INFO("setFrequencyReturn 0");
            setCommand=0;
            return 0;
        }
        else
        {
            ROS_INFO("setFrequencyReturn -1");
            return -1;
        }
    }
    else
    {
      ROS_INFO("setFrequencyReturn -1");
      return -1;
    }
}
void setBandrate(int set=0x05)
{
    unsigned char buf[6];
    buf[0]=0x68;
    buf[1]=0x05;
    buf[2]=address;
    buf[3]=0x0B;
    buf[4]=set;
    for(int i=1;i<5;i++)
    {
        buf[5]=(buf[5]+buf[i])&0xFF;
    }
    setCommand=2;
    write_Lock lck(mutex1);
    ser.write(buf,6);
    ROS_INFO("setBandrate");
}
int setBandrateReturn(std::vector<unsigned char>  buf,int size=6)
{
    int checksum=0;
    for(int i=1;i<size-1;i++)
    {
        checksum=(checksum+buf[i])&0xFF;
    }
    if((buf[0]==0x68)&&(buf[1]==0x05)&&(buf[2]==address)&&(buf[3]==0x8B)&&(buf[5]==checksum))
    {
        if(buf[4]==0x00)
        {
            ROS_INFO("setBandrateReturn 1");
            setCommand=0;
            return 1;
        }
        else if(buf[4]==0xFF)
        {
            ROS_INFO("setBandrateReturn 0");
            setCommand=0;
            return 0;
        }
        else
        {
            ROS_INFO("setBandrateReturn -1");
            return -1;
        }
    }
    else
    {
      ROS_INFO("setBandrateReturn -1");
      return -1;
    }
}


void setAddress(int set=0X00)
{
    unsigned char buf[6];
    buf[0]=0x68;
    buf[1]=0x05;
    buf[2]=address;
    buf[3]=0x0F;
    buf[4]=set;
    unsigned char cnt = 0;
    for(int i=1;i<5;i++)
    {
        cnt +=(buf[i]);
    }

    buf[5] = cnt;
    setCommand=3;
    write_Lock lck(mutex1);
    ser.write(buf,6);
    newaddress=set;
    ROS_INFO("setAddress");
}
int setAddressReturn(std::vector<unsigned char> buf,int size=6)
{
    int checksum=0;
    for(int i=1;i<size-1;i++)
    {
        checksum=(checksum+buf[i])&0xFF;
    }
    if((buf[0]==0x68)&&(buf[1]==0x05)&&(buf[2]==address)&&(buf[3]==0x8F)&&(buf[5]==checksum))
    {
        if(buf[4]==0x00)
        {
            address=newaddress;
            ROS_INFO("setAddressReturn 1");
            setCommand=0;
            return 1;
        }
        else if(buf[4]==0xFF)
        {
            ROS_INFO("setAddressReturn 0");
            setCommand=0;
            return 0;
        }
        else
        {
            ROS_INFO("setAddressReturn -1");
            return -1;
        }
    }
    else
    {
      ROS_INFO("setAddressReturn -1");
      return -1;
    }
}

void resetAngle()
{
    unsigned char buf[5];
    buf[0]=0x68;
    buf[1]=0x04;
    buf[2]=address;
    buf[3]=0x28;
    for(int i=1;i<4;i++)
    {
       buf[4]=(buf[4]+buf[i])&0xFF;
    }
    setCommand=4;
    write_Lock lck(mutex1);
    ser.write(buf,5);
    ROS_INFO("resetAngle");
}
int resetAngleReturn(std::vector<unsigned char>  buf,int size=6)
{
    int checksum=0;
    for(int i=1;i<size-1;i++)
    {
        checksum=(checksum+buf[i])&0xFF;
    }
    if((buf[0]==0x68)&&(buf[1]==0x05)&&(buf[2]==address)&&(buf[3]==0x28)&&(buf[5]==checksum))
    {
        if(buf[4]==0x00)
        {
            ROS_INFO("resetAngleReturn 1");
            setCommand=0;
            return 1;
        }
        else if(buf[4]==0xFF)
        {
            ROS_INFO("resetAngleReturn 0");
            setCommand=0;
            return 0;
        }
        else
        {
            ROS_INFO("resetAngleReturn -1");
            return -1;
        }
    }
    else
    {
      ROS_INFO("resetAngleReturn -1");
      return -1;
    }
}
void resetAcc()
{
    unsigned char buf[5];
    buf[0]=0x68;
    buf[1]=0x04;
    buf[2]=address;
    buf[3]=0x27;
    for(int i=1;i<4;i++)
    {
       buf[4]=(buf[4]+buf[i])&0xFF;
    }
    setCommand=5;
    write_Lock lck(mutex1);
    ser.write(buf,5);
    ROS_INFO("resetAcc");
}

int resetAccReturn(std::vector<unsigned char>  buf,int size=6)
{
    int checksum=0;
    for(int i=1;i<size-1;i++)
    {
        checksum=(checksum+buf[i])&0xFF;
    }
    if((buf[0]==0x68)&&(buf[1]==0x05)&&(buf[2]==address)&&(buf[3]==0x27)&&(buf[5]==checksum))
    {
        if(buf[4]==0x00)
        {
            ROS_INFO("resetAccReturn 1");
            setCommand=0;
            return 1;
        }
        else if(buf[4]==0xFF)
        {
            ROS_INFO("resetAccReturn 0");
            setCommand=0;
            return 0;
        }
        else
        {
            ROS_INFO("resetAccReturn -1");
            return -1;
        }
    }
    else
    {
      ROS_INFO("resetAccReturn -1");
      return -1;
    }
}

void setOutputKind(int set=0x70)
{
    unsigned char buf[6];
    buf[0]=0x68;
    buf[1]=0x05;
    buf[2]=address;
    buf[3]=0xFD;
    buf[4]=set;
    newkind=set-0x70;
    for(int i=1;i<5;i++)
    {
        buf[5]=(buf[5]+buf[i])&0xFF;
    }
    setCommand=6;
    write_Lock lck(mutex1);
    ser.write(buf,6);
    ROS_INFO("setOutputKind");
}

int setOutputKindReturn(std::vector<unsigned char>  buf,int size=6)
{
    int checksum=0;
    for(int i=1;i<size-1;i++)
    {
        checksum=(checksum+buf[i])&0xFF;
    }
    if((buf[0]==0x68)&&(buf[1]==0x05)&&(buf[2]==address)&&(buf[3]==0xFD)&&(buf[5]==checksum))
    {
        if(buf[4]==0x00)
        {
            ROS_INFO("setOutputKindReturn 1");
            kind=newkind;
            setCommand=0;
            return 1;
        }
        else if(buf[4]==0xFF)
        {
           ROS_INFO("setOutputKindReturn 0");
           setCommand=0;
           return 0;
        }
        else
        {
            ROS_INFO("setOutputKindReturn -1");
            return -1;
        }
    }
    else
    {
      ROS_INFO("setOutputKindReturn -1");
      return -1;
    }
}


int parseNineAxis(std::vector<unsigned char>  buf,int size=0x20)
{
    int checksum=0;
    unsigned int firsth,firstl,secondh,secondl,thirdh,thirdl;
    for(int i=1;i<size-1;i++)
    {
        checksum=(checksum+buf[i])&0xFF;
    }
    if((buf[0]==0x68)&&(buf[1]=0x1F)&&(buf[2]==address)&&(buf[3]==0x84)&&(buf[size-1]==checksum))
    {
        int sign;
        if((buf[4]&0x10)==0x10)
        {
            sign=-1;
        }
        else if((buf[4]&0x10)==0x00)
        {
            sign=1;
        }
        //firsth=(buf[4]&0xE0)>>4;
        firstl=(buf[4]&0x0F);
        secondh=(buf[5]&0xF0)>>4;
        secondl=(buf[5]&0x0F);
        thirdh=(buf[6]&0xF0)>>4;
        thirdl=(buf[6]&0x0F);
        roll=sign*(firstl*100+secondh*10+secondl+thirdh*0.1+thirdl*0.01);

        if((buf[7]&0x10)==0x10)
        {
            sign=-1;
        }
        else if((buf[7]&0x10)==0x00)
        {
            sign=1;
        }
        //firsth=(buf[7]&0xE0)>>4;
        firstl=(buf[7]&0x0F);
        secondh=(buf[8]&0xF0)>>4;
        secondl=(buf[8]&0x0F);
        thirdh=(buf[9]&0xF0)>>4;
        thirdl=(buf[9]&0x0F);
        pitch=sign*(firstl*100+secondh*10+secondl+thirdh*0.1+thirdl*0.01);

        if((buf[10]&0x10)==0x10)
        {
            sign=-1;
        }
        else if((buf[10]&0x10)==0x00)
        {
            sign=1;
        }
        //firsth=(buf[10]&0xE0)>>4;
        firstl=(buf[10]&0x0F);
        secondh=(buf[11]&0xF0)>>4;
        secondl=(buf[11]&0x0F);
        thirdh=(buf[12]&0xF0)>>4;
        thirdl=(buf[12]&0x0F);
        yaw=sign*(firstl*100+secondh*10+secondl+thirdh*0.1+thirdl*0.01);

        if((buf[13]&0x10)==0x10)
        {
            sign=-1;
        }
        else if((buf[13]&0x10)==0x00)
        {
            sign=1;
        }
        //firsth=(buf[13]&0xE0)>>4;
        firstl=(buf[13]&0x0F);
        secondh=(buf[14]&0xF0)>>4;
        secondl=(buf[14]&0x0F);
        thirdh=(buf[15]&0xF0)>>4;
        thirdl=(buf[15]&0x0F);
        accx=sign*(firstl*10+secondh*1+secondl*0.1+thirdh*0.01+thirdl*0.001);

        if((buf[16]&0x10)==0x10)
        {
            sign=-1;
        }
        else if((buf[16]&0x10)==0x00)
        {
            sign=1;
        }
        //firsth=(buf[16]&0xE0)>>4;
        firstl=(buf[16]&0x0F);
        secondh=(buf[17]&0xF0)>>4;
        secondl=(buf[17]&0x0F);
        thirdh=(buf[18]&0xF0)>>4;
        thirdl=(buf[18]&0x0F);
        accy=sign*(firstl*10+secondh*1+secondl*0.1+thirdh*0.01+thirdl*0.001);

        if((buf[19]&0x10)==0x10)
        {
            sign=-1;
        }
        else if((buf[19]&0x10)==0x00)
        {
            sign=1;
        }
        //firsth=(buf[19]&0xE0)>>4;
        firstl=(buf[19]&0x0F);
        secondh=(buf[20]&0xF0)>>4;
        secondl=(buf[20]&0x0F);
        thirdh=(buf[21]&0xF0)>>4;
        thirdl=(buf[21]&0x0F);
        accz=sign*(firstl*10+secondh*1+secondl*0.1+thirdh*0.01+thirdl*0.001);

        if((buf[22]&0x10)==0x10)
        {
            sign=-1;
        }
        else if((buf[22]&0x10)==0x00)
        {
            sign=1;
        }
        //firsth=(buf[22]&0xE0)>>4;
        firstl=(buf[22]&0x0F);
        secondh=(buf[23]&0xF0)>>4;
        secondl=(buf[23]&0x0F);
        thirdh=(buf[24]&0xF0)>>4;
        thirdl=(buf[24]&0x0F);
        gyrox=sign*(firstl*100+secondh*10+secondl+thirdh*0.1+thirdl*0.01);

        if((buf[25]&0x10)==0x10)
        {
            sign=-1;
        }
        else if((buf[25]&0x10)==0x00)
        {
            sign=1;
        }
        //firsth=(buf[25]&0xE0)>>4;
        firstl=(buf[25]&0x0F);
        secondh=(buf[26]&0xF0)>>4;
        secondl=(buf[26]&0x0F);
        thirdh=(buf[27]&0xF0)>>4;
        thirdl=(buf[27]&0x0F);
        gyroy=sign*(firstl*100+secondh*10+secondl+thirdh*0.1+thirdl*0.01);
        if((buf[28]&0x10)==0x10)
        {
            sign=-1;
        }
        else if((buf[28]&0x10)==0x00)
        {
            sign=1;
        }
        //firsth=(buf[28]&0xE0)>>4;
        firstl=(buf[28]&0x0F);
        secondh=(buf[29]&0xF0)>>4;
        secondl=(buf[29]&0x0F);
        thirdh=(buf[30]&0xF0)>>4;
        thirdl=(buf[30]&0x0F);
        gyroz=sign*(firstl*100+secondh*10+secondl+thirdh*0.1+thirdl*0.01);
        //ROS_INFO("parseNineAxis 1");
        sensor_msgs::Imu imudata;
        imudata.header.frame_id="imu_link";
        imudata.header.stamp=ros::Time::now();
        imudata.angular_velocity.x=gyroy*3.14159265359/180;
        imudata.angular_velocity.y=-1*gyrox*3.14159265359/180;
        imudata.angular_velocity.z=gyroz*3.14159265359/180;

        //imudata.linear_acceleration.x=0;
        imudata.linear_acceleration.x=accy*9.806649999788;
        imudata.linear_acceleration.y=-1*accx*9.806649999788;
        imudata.linear_acceleration.z=accz*9.806649999788;

        /*tf::Matrix3x3 rotation_matrix;
        rotation_matrix.setEulerYPR(yaw*3.1415926/180,pitch*3.1415926*180,roll*3.1415926/180);
        tf::Quaternion quat;
        rotation_matrix.getRotation(quat);*/
        //std::cout<<"rpy:"<<roll<<","<<pitch<<","<<yaw<<std::endl;
        //ROS_INFO("here!!!");
        //ROS_INFO("yaw = %2f",yaw);
        tf::Quaternion quat=tf::createQuaternionFromRPY(pitch*3.14159265359/180.0, -1*roll*3.14159265359/180.0, yaw*3.14159265359/180.0);
        imudata.orientation.x=quat.getX();
        imudata.orientation.y=quat.getY();
        imudata.orientation.z=quat.getZ();
        imudata.orientation.w=quat.getW();
        /*
        imudata.orientation_covariance[0]=0.0012729111009278221;
        imudata.orientation_covariance[4]=0.0012729111009278221;
        imudata.orientation_covariance[8]=0;

        imudata.angular_velocity_covariance[0]=0.0025000000000000005;
        imudata.angular_velocity_covariance[4]=0.0025000000000000005;
        imudata.angular_velocity_covariance[8]=0.000225;

        imudata.linear_acceleration_covariance[0]=0.12249999999999998;
        imudata.linear_acceleration_covariance[4]=0.12249999999999998;
        imudata.linear_acceleration_covariance[8]=0.09;
        */

        imu_pub.publish(imudata);


        return 1;
    }
    else
    {
      ROS_INFO("parseNineAxis -1");
      return -1;
    }
}


int parsestandard1(std::vector<unsigned char>  buf,int size=0x0E)
{
    int checksum=0;
    unsigned int firsth,firstl,secondh,secondl,thirdh,thirdl;
    for(int i=1;i<size-1;i++)
    {
       checksum=(checksum+buf[i])&0xFF;
    }
    if((buf[0]==0x68)&&(buf[1]=0x0D)&&(buf[2]==address)&&(buf[3]==0x84)&&(buf[size-1]==checksum))
    {
        int sign;
        if((buf[4]&0x10)==0x10)
        {
            sign=-1;
        }
        else if((buf[4]&0x10)==0x00)
        {
            sign=1;
        }
        //firsth=(buf[4]&0xE0)>>4;
        firstl=(buf[4]&0x0F);
        secondh=(buf[5]&0xF0)>>4;
        secondl=(buf[5]&0x0F);
        thirdh=(buf[6]&0xF0)>>4;
        thirdl=(buf[6]&0x0F);
        gyroz=sign*(firstl*100+secondh*10+secondl+thirdh*0.1+thirdl*0.01);

        if((buf[7]&0x10)==0x10)
        {
            sign=-1;
        }
        else if((buf[7]&0x10)==0x00)
        {
            sign=1;
        }
        //firsth=(buf[7]&0xE0)>>4;
        firstl=(buf[7]&0x0F);
        secondh=(buf[8]&0xF0)>>4;
        secondl=(buf[8]&0x0F);
        thirdh=(buf[9]&0xF0)>>4;
        thirdl=(buf[9]&0x0F);
        accy=sign*(firstl*10+secondh*1+secondl*0.1+thirdh*0.01+thirdl*0.001);


        if((buf[10]&0x10)==0x10)
        {
            sign=-1;
        }
        else if((buf[10]&0x10)==0x00)
        {
            sign=1;
        }
        //firsth=(buf[10]&0xE0)>>4;
        firstl=(buf[10]&0x0F);
        secondh=(buf[11]&0xF0)>>4;
        secondl=(buf[11]&0x0F);
        thirdh=(buf[12]&0xF0)>>4;
        thirdl=(buf[12]&0x0F);
        yaw=sign*(firstl*100+secondh*10+secondl+thirdh*0.1+thirdl*0.01);
        ROS_INFO("parsestandard1 1");
        sensor_msgs::Imu imudata;
        imudata.header.frame_id="imu";
        imudata.header.stamp=ros::Time::now();
        imudata.angular_velocity.x=0;
        imudata.angular_velocity.y=0;
        imudata.angular_velocity.z=gyroz*3.1415926/180;
        imudata.linear_acceleration.x=accy*9.806649999788;
        imudata.linear_acceleration.y=0;
        imudata.linear_acceleration.z=0;
        /*tf::Matrix3x3 rotation_matrix;
        rotation_matrix.setEulerYPR(yaw*3.1415926/180,0,0);
        tf::Quaternion quat;
        rotation_matrix.getRotation(quat);*/
        tf::Quaternion quat=tf::createQuaternionFromRPY(0, 0, yaw*3.14159265359/180.0);

        imudata.orientation.x=quat.getX();
        imudata.orientation.y=quat.getY();
        imudata.orientation.z=quat.getZ();
        imudata.orientation.w=quat.getW();
        imu_pub.publish(imudata);
        return 1;
    }
    else
    {
      ROS_INFO("parsestandard1 -1");
      return -1;
    }
}

int parsestandard2(std::vector<unsigned char> buf,int size=0x0E)
{
    int checksum=0;
    unsigned int firsth,firstl,secondh,secondl,thirdh,thirdl;
    for(int i=1;i<size-1;i++)
    {
        checksum=(checksum+buf[i])&0xFF;
    }
    if((buf[0]==0x68)&&(buf[1]=0x0D)&&(buf[2]==address)&&(buf[3]==0x84)&&(buf[size-1]==checksum))
    {
        int sign;
        if((buf[4]&0x10)==0x10)
        {
            sign=-1;
        }
        else if((buf[4]&0x10)==0x00)
        {
            sign=1;
        }
        //firsth=(buf[4]&0xE0)>>4;
        firstl=(buf[4]&0x0F);
        secondh=(buf[5]&0xF0)>>4;
        secondl=(buf[5]&0x0F);
        thirdh=(buf[6]&0xF0)>>4;
        thirdl=(buf[6]&0x0F);
        accx=sign*(firstl*10+secondh*1+secondl*0.1+thirdh*0.01+thirdl*0.001);

        if((buf[7]&0x10)==0x10)
        {
            sign=-1;
        }
        else if((buf[7]&0x10)==0x00)
        {
            sign=1;
        }
        //firsth=(buf[7]&0xE0)>>4;
        firstl=(buf[7]&0x0F);
        secondh=(buf[8]&0xF0)>>4;
        secondl=(buf[8]&0x0F);
        thirdh=(buf[9]&0xF0)>>4;
        thirdl=(buf[9]&0x0F);
        accy=sign*(firstl*10+secondh*1+secondl*0.1+thirdh*0.01+thirdl*0.001);


        if((buf[10]&0x10)==0x10)
        {
            sign=-1;
        }
        else if((buf[10]&0x10)==0x00)
        {
            sign=1;
        }
        //firsth=(buf[10]&0xE0)>>4;
        firstl=(buf[10]&0x0F);
        secondh=(buf[11]&0xF0)>>4;
        secondl=(buf[11]&0x0F);
        thirdh=(buf[12]&0xF0)>>4;
        thirdl=(buf[12]&0x0F);
        yaw=sign*(firstl*100+secondh*10+secondl+thirdh*0.1+thirdl*0.01);
        ROS_INFO("parsestandard2 1");
        sensor_msgs::Imu imudata;
        imudata.header.frame_id="imu";
        imudata.header.stamp=ros::Time::now();
        imudata.angular_velocity.x=0;
        imudata.angular_velocity.y=0;
        imudata.angular_velocity.z=0;
        imudata.linear_acceleration.x=accy*9.806649999788;
        imudata.linear_acceleration.y=-1*accx*9.806649999788;
        imudata.linear_acceleration.z=0;
        /*tf::Matrix3x3 rotation_matrix;
        rotation_matrix.setEulerYPR(yaw*3.1415926/180,0,0);
        tf::Quaternion quat;
        rotation_matrix.getRotation(quat);*/
        tf::Quaternion quat=tf::createQuaternionFromRPY(0,0, yaw*3.14159265359/180.0);

        imudata.orientation.x=quat.getX();
        imudata.orientation.y=quat.getY();
        imudata.orientation.z=quat.getZ();
        imudata.orientation.w=quat.getW();
        imu_pub.publish(imudata);
        return -1;
    }
    else
    {
      ROS_INFO("parsestandard2 -1");
      return -1;
    }
}


int parsestandard3(std::vector<unsigned char> buf,int size=0x11)
{
    int checksum=0;
    unsigned int firsth,firstl,secondh,secondl,thirdh,thirdl;
    for(int i=1;i<size-1;i++)
    {
        checksum=(checksum+buf[i])&0xFF;
    }
    if((buf[0]==0x68)&&(buf[1]=0x10)&&(buf[2]==address)&&(buf[3]==0x84)&&(buf[size-1]==checksum))
    {
        int sign;
        if((buf[4]&0x10)==0x10)
        {
            sign=-1;
        }
        else if((buf[4]&0x10)==0x00)
        {
            sign=1;
        }
        //firsth=(buf[4]&0xE0)>>4;
        firstl=(buf[4]&0x0F);
        secondh=(buf[5]&0xF0)>>4;
        secondl=(buf[5]&0x0F);
        thirdh=(buf[6]&0xF0)>>4;
        thirdl=(buf[6]&0x0F);
        gyroz=sign*(firstl*100+secondh*10+secondl+thirdh*0.1+thirdl*0.01);

        if((buf[7]&0x10)==0x10)
        {
            sign=-1;
        }
        else if((buf[7]&0x10)==0x00)
        {
            sign=1;
        }
        //firsth=(buf[7]&0xE0)>>4;
        firstl=(buf[7]&0x0F);
        secondh=(buf[8]&0xF0)>>4;
        secondl=(buf[8]&0x0F);
        thirdh=(buf[9]&0xF0)>>4;
        thirdl=(buf[9]&0x0F);
        accx=sign*(firstl*10+secondh*1+secondl*0.1+thirdh*0.01+thirdl*0.001);


        if((buf[10]&0x10)==0x10)
        {
            sign=-1;
        }
        else if((buf[10]&0x10)==0x00)
        {
            sign=1;
        }
        //firsth=(buf[10]&0xE0)>>4;
        firstl=(buf[10]&0x0F);
        secondh=(buf[11]&0xF0)>>4;
        secondl=(buf[11]&0x0F);
        thirdh=(buf[12]&0xF0)>>4;
        thirdl=(buf[12]&0x0F);
        accy=sign*(firstl*10+secondh*1+secondl*0.1+thirdh*0.01+thirdl*0.001);


        if((buf[13]&0x10)==0x10)
        {
            sign=-1;
        }
        else if((buf[13]&0x10)==0x00)
        {
            sign=1;
        }
        //firsth=(buf[13]&0xE0)>>4;
        firstl=(buf[13]&0x0F);
        secondh=(buf[14]&0xF0)>>4;
        secondl=(buf[14]&0x0F);
        thirdh=(buf[15]&0xF0)>>4;
        thirdl=(buf[15]&0x0F);
        yaw=sign*(firstl*100+secondh*10+secondl+thirdh*0.1+thirdl*0.01);
        ROS_INFO("parsestandard3 1");
        sensor_msgs::Imu imudata;
        imudata.header.frame_id="imu";
        imudata.header.stamp=ros::Time::now();
        imudata.angular_velocity.x=0;
        imudata.angular_velocity.y=0;
        imudata.angular_velocity.z=gyroz*3.1415926/180;
        imudata.linear_acceleration.x=accy*9.806649999788;
        imudata.linear_acceleration.y=-1*accx*9.806649999788;
        imudata.linear_acceleration.z=0;
        /*tf::Matrix3x3 rotation_matrix;
        rotation_matrix.setEulerYPR(yaw*3.1415926/180,0,0);
        tf::Quaternion quat;
        rotation_matrix.getRotation(quat);*/
        tf::Quaternion quat=tf::createQuaternionFromRPY(0,0, yaw*3.14159265359/180.0);
        imudata.orientation.x=quat.getX();
        imudata.orientation.y=quat.getY();
        imudata.orientation.z=quat.getZ();
        imudata.orientation.w=quat.getW();
        imudata.orientation_covariance = boost::array<double, 9>({
            1e6, 0.0, 0.0,0.0, 1e6, 0.0,0.0, 0.0, 1e-6});
        imudata.angular_velocity_covariance = boost::array<double, 9>({
            1e6, 0.0, 0.0,0.0, 1e6, 0.0,0.0, 0.0, 1e6});
        imudata.linear_acceleration_covariance = boost::array<double, 9>({
            1e6, 0.0, 0.0,0.0, 1e6, 0.0,0.0, 0.0, 1e6});
        imu_pub.publish(imudata);
        return -1;
    }
    else
    {
      ROS_INFO("parsestandard3 -1");
      return -1;
    }
}

void process()
{
  ros::Rate rate(200);
  std::vector<unsigned char> buf;
  int returnValue;
  int command;
  int length;
  int avail_length;
  int front=0;
  std::vector<unsigned char> uartData;
  while(ros::ok())
  {

    avail_length=ser.available();
    if(avail_length)
    {
      ser.read(uartData,avail_length);
      /*
      std::cout<<"SIZE:"<<uartData.size()<<std::endl; 
      int n =0;
      while(n < uartData.size())     
      {
         ROS_INFO("uartData:%d",uartData[n]);
         n++;
      }
      */
      if(front<uartData.size()-1)//读取到帧头与长度
      {
        while((uartData[front]!=0x68)&&(uartData.size()-1))
        {
          front++;
        }
        if(uartData[front]==0x68)
        {
          length=uartData[front+1];

          if(length<uartData.size()-front-1)//包含完整数据帧
          {
            command=uartData[front+3];
            buf.clear();
            buf.resize(length+1);
            std::copy(uartData.begin()+front,uartData.begin()+front+length+1,buf.begin());//将完整数据帧拷贝到buf中
            
            //ROS_INFO("COMMAND:%d,length:%d",command,length);

            if((command==0x8C)&&(length==5))
            {
              returnValue=setFrequencyReturn(buf);
            }
            else if((command==0x8B)&&(length==5))
            {
              returnValue=setBandrateReturn(buf);
            }
            else if((command==0x8F)&&(length==5))
            {
              returnValue=setAddressReturn(buf);
            }
            else if((command==0x28)&&(length==5))
            {
              ROS_INFO("GET resetAngleReturn");
              returnValue=resetAngleReturn(buf);
            }
            else if((command==0x27)&&(length==5))
            {
              ROS_INFO("GET resetAccReturn");
              returnValue=resetAccReturn(buf);
            }
            else if((command==0xFD)&&(length==5))
            {
              returnValue=setOutputKindReturn(buf);
            }
            else if(command==0x84)
            {
              if((kind==0)&&(length==0x1F))
              {
                returnValue=parseNineAxis(buf);
              }
              else if((kind==1)&&(length==0X0D))
              {
                returnValue=parsestandard1(buf);
              }
              else if((kind==2)&&(length==0X0D))
              {
                returnValue=parsestandard2(buf);
              }
              else if((kind==3)&&(length==0X10))
              {
                returnValue=parsestandard3(buf);
              }
              else //解析数据不对
              {
                front++;
              }
            }
            else //解析数据不对
            {
              front++;
            }
            if(returnValue==-1)//解析数据不对
            {
              front++;
            }
            else //清除缓冲区已处理数据
            {
              uartData.erase(uartData.begin(),uartData.begin()+front+length+1);//删除错误及已处理数据
              front=0;
            }
          }
          else
          {
            continue;//等待读取到完整数据帧
          }
        }
        else
        {
          continue;//等待读取到帧头与长度
        }
      }
      else
      {
          continue;//等待读取到帧头与长度
      }
    }
    //ros::spinOnce();
    rate.sleep();
  }
}

void imu_pose_init_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  cout<<"imu angel and acc reset ..."<< endl;
  //nsigned char sendbuf[5] = {0x68, 0x04, 0x00, 0x27, 0x2b};
  //write_Lock lck(mutex1);
  //ser.write(sendbuf,5);

  //resetAngle();
  //sleep(20);
  resetAcc();
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"IMU_node");
    ros::NodeHandle nh;
  //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    ser.setPort("/dev/imu");
    //设置串口通信的波特率
    ser.setBaudrate(115200);
    //串口设置timeout
    ser.setTimeout(to);

    init_time=clock();
    current_time=clock();

    current_time=init_time;
    g_last_imu_time=current_time;

    //imu_angular_vel={ 0, 0, imu_angular_vel_rz};
    //imu_linear_acc={const_imu_linear_acc_x, const_imu_linear_acc_y,0};
/*
    imu_angular_vel[0] = 0;
    imu_angular_vel[1] = 0;
    imu_angular_vel[2] = imu_angular_vel_rz;

    imu_linear_acc[0] = const_imu_linear_acc_x;
    imu_linear_acc[1] = const_imu_linear_acc_y;
    imu_linear_acc[2] = 0;
*/
    try
    {
        //打开串口
        ser.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

    //判断串口是否打开成功
    if(ser.isOpen())
    {
        ROS_INFO_STREAM("/dev/IMU is opened.");
        //ser.flushInput();      //清空输入缓存,把多余的无用数据删除
        sleep(0.1);            //延时0.1秒,确保有数据进入
    }
    else
    {
        ROS_ERROR_STREAM("/dev/IMU is close.");
        return -1;
    }

    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);

    imu_pub=nh.advertise<sensor_msgs::Imu>("/imu/data2", 1);
    boost::thread threadRead(&process);

    imu_pose_init_sub = nh.subscribe("initialpose", 2, &imu_pose_init_callback);

    setFrequency(6);  //100hz
    
    while(setCommand!=0)
    {
          sleep(1);
    }

    ROS_INFO("Set TL740D 100Hz");

    ros::spin();
/*
    setAddress(1);
    while(setCommand!=0)
    {
          sleep(1);
    }
    
    ROS_INFO("Set address 1");*/
    //ros::spin();
/*
    setBandrate();
    while(setCommand!=0)
    {
          sleep(1);
    }
    setAddress();
    while(setCommand!=0)
    {
          sleep(1);
    }
    resetAngle();
    while(setCommand!=0)
    {
          sleep(1);
    }
    resetAcc();
    while(setCommand!=0)
    {
          sleep(1);
    }
    setOutputKind();
    while(setCommand!=0)
    {
          sleep(1);
    }
    ROS_INFO("OVER!");
    */
    threadRead.join();
    return 0;
}
