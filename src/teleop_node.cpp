#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include <sensor_msgs/Joy.h>
#include <string.h>
#include <iostream>
#include <std_srvs/Empty.h>

#include <math.h>

using namespace std;


//mm
double xx = 590,yy = 0,zz = 0,step_ = 0.4,gripper_ = 0; 
double r1 = 590; 
double step_deg = 0.004; //rad
double L0=0,L1=360,L2=70,L3=230;

// joint limits [rad]
double base_min=-3.14159*2,   base_max=3.14159*2;
double shoulder_min=-0.25,    shoulder_max=3.14159/2;
double elbow_min=-3.14159*2,   elbow_max=3.14159*2;
double gripp_min=87, gripp_ref =90,   gripp_max=93;

//others
double XX,YY,ZZ,GO,GC,GSum,GRes,Home_,Sleep_;


class TeleopArm
{
public:
  TeleopArm();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  int movX_,movY_,movZ_,gripp_o,gripp_c,gripp_s,gripp_r;
  int start_,stop_;

  ros::NodeHandle nh2_;
  ros::Subscriber joy_sub_;

};


TeleopArm::TeleopArm():
  movX_(0),movY_(3),movZ_(1),
  gripp_o(4),gripp_c(5),
  gripp_s(6),gripp_r(7),
  start_(9),stop_(8)
  {

  nh2_.param("mov_X", movX_, movX_);
  nh2_.param("mov_Y", movY_, movY_);
  nh2_.param("mov_Z", movZ_, movZ_);
  nh2_.param("open_gr", gripp_o, gripp_o);
  nh2_.param("close_gr", gripp_c, gripp_c);
  nh2_.param("gr_0_sum", gripp_s, gripp_s);
  nh2_.param("gr_0_res", gripp_r, gripp_r);
  nh2_.param("strt_", start_, start_);
  nh2_.param("slp_", stop_, stop_);

  joy_sub_ = nh2_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopArm::joyCallback, this);

}

void TeleopArm::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  XX = joy->axes[movX_];
  YY = joy->axes[movY_];
  ZZ = joy->axes[movZ_];
  GO = joy->buttons[gripp_o];
  GC = joy->buttons[gripp_c];
  GSum = joy->buttons[gripp_s];
  GRes = joy->buttons[gripp_r];
  Home_ = joy->buttons[start_];
  Sleep_ = joy->buttons[stop_];
}


int main(int argc, char** argv)
{
  double r1,r2,cos_th3;
  double th1=0,th2=0,th3=0;
  double thf=0;
  double th1_ant,th2_ant,th3_ant,xx_ant,zz_ant,yy_ant,gripp_; 
  double thf_ant;
  string th1_str,th2_str,th3_str;
  
  std_msgs::Float64 th1_;
  std_msgs::Float64 th2_;
  std_msgs::Float64 th3_;
  std_msgs::Float64 thf_;
  std_msgs::Float32 gripper_;
  geometry_msgs::Point xyz_d;
  geometry_msgs::Pose joints_vec;

  ros::init(argc, argv, "teleop_arm");
  ros::NodeHandle nh_;
  ros::Rate loop_rate(50);

  TeleopArm teleop_arm;

  ros::Publisher jointBase_pub_;
  ros::Publisher jointShoulder_pub_;
  ros::Publisher jointElbow_pub_;
  ros::Publisher jointFinger1_pub_;
  ros::Publisher jointFinger2_pub_;
  ros::Publisher joints_pub;
  ros::Publisher xyz_pub;
  ros::Publisher gripper_pub_;

 
  jointBase_pub_ = nh_.advertise<std_msgs::Float64>("/msv_3dof_arm/joint_base_position_controller/command",10);
  jointShoulder_pub_ = nh_.advertise<std_msgs::Float64>("/msv_3dof_arm/joint_shoulder_position_controller/command",10);
  jointElbow_pub_ = nh_.advertise<std_msgs::Float64>("/msv_3dof_arm/joint_elbow_position_controller/command",10);
  jointFinger1_pub_ = nh_.advertise<std_msgs::Float64>("/msv_3dof_arm/joint_finger1_position_controller/command",10);
  jointFinger2_pub_ = nh_.advertise<std_msgs::Float64>("/msv_3dof_arm/joint_finger2_position_controller/command",10);
  joints_pub = nh_.advertise<geometry_msgs::Pose>("/msv_3dof_arm/joints_vector",1);
  xyz_pub = nh_.advertise<geometry_msgs::Point>("/msv_3dof_arm/xyz",1);

  gripper_pub_ = nh_.advertise<std_msgs::Float32>("/gripper_moteus",1);

  //Sleep position
  th1 = 0;
  xx = 130.35;
  yy = 0;
  zz = -10.2;

  while(ros::ok()){
    th1_ant = th1;
    th2_ant = th2;
    th3_ant = th3;
    xx_ant = xx;
    yy_ant = yy;
    zz_ant = zz;

    if(Home_==1){
      th1 = -3.14159;
      th2 = 1.0731;
      th3 = -2.2973;
      xx = 250;
      yy = 0;
      zz = 100;
    }

    if(Sleep_==1){
      th1 = 0;
      xx = 130.35;
      yy = 0;
      zz = -10.2;
    }

    if(XX<-0.8){
      xx+=step_;
    }
    else{
      if(XX>0.8)
        xx-=step_;
    }

    if(YY>0.8){
      th1+=step_deg;
    }
    else{
      if(YY<-0.8)
        th1-=step_deg;
    }

    if(ZZ>0.8){
      zz+=step_;
    }
    else{
      if(ZZ<-0.8)
        zz-=step_;
    }

    gripp_=gripp_ref;
    thf_ant = thf;
    if (GO==1){
      gripp_ = gripp_ref+6;
      thf+=step_deg;
    }
    if (GC==1){
      gripp_ = gripp_ref-6;
      thf-=step_deg;
    }
    if (GSum==1){
      gripp_ref = gripp_ref+0.008;
    }
    if (GRes==1){
      gripp_ref = gripp_ref-0.008;
    }


    // ********* INVERSE KINEMATICS *********
    r1 = xx;//sqrt(xx*xx+yy*yy);
    r2 = zz;

    /*
    if (xx!=0 && yy!=0){
      th1 = atan2(yy,xx) + asin(L2/sqrt(xx*xx+yy*yy));
    }
    else{
      th1= 0;
    }
    */

    cos_th3 = (r1*r1 + r2*r2 - L1*L1 - L3*L3)/(2*L1*L3);

    if(abs(cos_th3 - 1) <= 0.0001 ){
      th3 = 0;
    }
    else{
      th3 = atan2(-sqrt(1-cos_th3*cos_th3),cos_th3);
    }

    th2 = atan2(r2,r1) - atan2(L3*sin(th3),L1+(L3*cos_th3));


    th1_str=to_string(th1);
    char const *th1_char = th1_str.c_str();
    th2_str=to_string(th2);
    char const *th2_char = th2_str.c_str();
    th3_str=to_string(th3);
    char const *th3_char = th3_str.c_str();


    if(
      th1<base_min || th1>base_max|| strcmp(th1_char,"nan")==0|| strcmp(th1_char,"-nan")==0||
      th2<shoulder_min || th2>shoulder_max || strcmp(th2_char,"nan")==0|| strcmp(th2_char,"-nan")==0||
      th3<elbow_min || th3>elbow_max || strcmp(th3_char,"nan")==0 || strcmp(th3_char,"-nan")==0
      ){
      th1 = th1_ant;
      th2 = th2_ant;
      th3 = th3_ant;
      xx = xx_ant;
      yy = yy_ant;
      zz = zz_ant;

    }

    

    th1_.data = th1 + 3.14159;
    th2_.data = th2;
    th3_.data = th3;
    
    joints_vec.orientation.x = th1;
    joints_vec.orientation.y = th2;
    joints_vec.orientation.z = th3;
    gripp_ = gripp_*10/180+2; //duty cycle for Rpi GPIO PWM
    gripper_.data = gripp_;
    joints_vec.orientation.w = gripp_;

    xyz_d.x = xx;
    xyz_d.y = yy;
    xyz_d.z = zz;


    jointBase_pub_.publish(th1_);
    jointShoulder_pub_.publish(th2_);
    jointElbow_pub_.publish(th3_);
    gripper_pub_.publish(gripper_);
    joints_pub.publish(joints_vec);
    xyz_pub.publish(xyz_d);

    thf_.data = thf;
    jointFinger1_pub_.publish(thf_);
    thf_.data = -thf;
    jointFinger2_pub_.publish(thf_);

    ros::spinOnce();                 
    loop_rate.sleep();
  }

}