#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include <string.h>
#include <iostream>
#include <std_srvs/Empty.h>

#include "geometry_msgs/Pose.h"

#include <math.h>

std_msgs::Float32 th1_;
std_msgs::Float32 th2_;
std_msgs::Float32 th3_;
std_msgs::Float32 th4_;

void base_joint_Callback(const std_msgs::Float32::ConstPtr& data_)
{
  th1_=*data_; 
}

void shoulder_joint_Callback(const std_msgs::Float32::ConstPtr& data_)
{
  th2_=*data_; 
}

void elbow_joint_Callback(const std_msgs::Float32::ConstPtr& data_)
{
  th3_=*data_; 
}

void finger_joint_Callback(const std_msgs::Float32::ConstPtr& data_)
{
  th4_=*data_; 
}

int main(int argc, char** argv)
{
  
  th1_.data=0;
  th2_.data=0;
  th3_.data=0;
  th4_.data=0;

  std_msgs::Float64 th_base;
  std_msgs::Float64 th_shoulder;
  std_msgs::Float64 th_elbow;
  std_msgs::Float64 th_finger;

  ros::init(argc, argv, "gazebo_joint_controllers");
  ros::NodeHandle nh_;
  ros::Rate loop_rate(100);

  ros::Publisher jointBase_pub_;
  ros::Publisher jointShoulder_pub_;
  ros::Publisher jointElbow_pub_;
  ros::Publisher jointFinger1_pub_;
  ros::Publisher jointFinger2_pub_;

  // Disable physics
  ros::ServiceClient pauseGazeboPhysics = nh_.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
  std_srvs::Empty pauseSrv;
  pauseGazeboPhysics.call(pauseSrv);
  ros::spinOnce(); 
 
  jointBase_pub_ = nh_.advertise<std_msgs::Float64>("/msv_3dof_arm/joint_base_position_controller/command",1);
  jointShoulder_pub_ = nh_.advertise<std_msgs::Float64>("/msv_3dof_arm/joint_shoulder_position_controller/command",1);
  jointElbow_pub_ = nh_.advertise<std_msgs::Float64>("/msv_3dof_arm/joint_elbow_position_controller/command",1);
  jointFinger1_pub_ = nh_.advertise<std_msgs::Float64>("/msv_3dof_arm/joint_finger1_position_controller/command",1);
  jointFinger2_pub_ = nh_.advertise<std_msgs::Float64>("/msv_3dof_arm/joint_finger2_position_controller/command",1);

  ros::Subscriber sub_base = nh_.subscribe("/joint_base_moteus", 1, base_joint_Callback);
  ros::Subscriber sub_shoulder = nh_.subscribe("/joint_shoulder_moteus", 1, shoulder_joint_Callback);
  ros::Subscriber sub_elbow = nh_.subscribe("/joint_elbow_moteus", 1, elbow_joint_Callback);
  ros::Subscriber sub_finger = nh_.subscribe("/joint_finger_moteus", 1, finger_joint_Callback);

  pauseGazeboPhysics.call(pauseSrv);
  ros::spinOnce(); 

  while(ros::ok()){
    
    th_base.data = th1_.data;
    th_shoulder.data = th2_.data;
    th_elbow.data = th3_.data;

    jointBase_pub_.publish(th_base);
    jointShoulder_pub_.publish(th_shoulder);
    jointElbow_pub_.publish(th_elbow);

    th_finger.data = th4_.data;
    jointFinger1_pub_.publish(th_finger);
    th_finger.data = -th4_.data;
    jointFinger2_pub_.publish(th_finger);

    ros::spinOnce();                 
    loop_rate.sleep();
  }

}