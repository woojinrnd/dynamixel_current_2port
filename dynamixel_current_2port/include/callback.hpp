#ifndef CALLBACK_H
#define CALLBACK_H

#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <boost/thread.hpp>

#include "dynamixel.hpp"
#include "Walkingpattern_generator.hpp"

#include "dynamixel_current_2port/Select_Motion.h"
#include "dynamixel_current_2port/Turn_Angle.h"
#include "dynamixel_current_2port/UD_NeckAngle.h"
#include "dynamixel_current_2port/RL_NeckAngle.h"
#include "dynamixel_current_2port/Emergency.h"




using Eigen::VectorXd;

class Callback
{
public:
  // Callback();
  Callback(Trajectory *trajectoryPtr, IK_Function *IK_Ptr, Dxl *dxlPtr);

  Trajectory *trajectoryPtr;
  IK_Function *IK_Ptr;
  Dxl *dxlPtr;

  
  ros::NodeHandle nh;
  // Function
  virtual void JointStatesCallback(const sensor_msgs::JointState::ConstPtr &joint_command);
  virtual void FSRsensorCallback(const std_msgs::UInt8::ConstPtr &FSR);
  virtual void IMUsensorCallback(const sensor_msgs::Imu::ConstPtr &IMU);
  virtual void Check_FSR();

  
  // virtual void SelectMotion(const std_msgs::UInt8::ConstPtr &msg);

  virtual void Write_Leg_Theta();
  virtual void Write_Arm_Theta();

  sensor_msgs::JointState joint_state;


  //////////////////////////////Callback Thread

  // Client (재민이형 코드에 들어감)
  ros::ServiceClient client_SM = nh.serviceClient<dynamixel_current_2port::Select_Motion>("/Move_decision/Select_Motion");
  ros::ServiceClient client_TA = nh.serviceClient<dynamixel_current_2port::Turn_Angle>("/Move_decision/Turn_Angle");
  ros::ServiceClient client_UD_Neck = nh.serviceClient<dynamixel_current_2port::UD_NeckAngle>("/Move_decision/UD_NeckAngle");
  ros::ServiceClient client_RL_Neck = nh.serviceClient<dynamixel_current_2port::RL_NeckAngle>("/Move_decision/RL_NeckAngle");
  ros::ServiceClient client_Emergency = nh.serviceClient<dynamixel_current_2port::Emergency>("/Move_decision/Emergency");


  dynamixel_current_2port::Select_Motion srv_SM;
  dynamixel_current_2port::Turn_Angle srv_TA;
  dynamixel_current_2port::UD_NeckAngle srv_UD_Neck;
  dynamixel_current_2port::RL_NeckAngle srv_RL_Neck;
  dynamixel_current_2port::Emergency srv_Emergency;


  /////////Service callbacek
  virtual void SelectMotion();
  virtual void Move_UD_NeckAngle();
  virtual void Move_RL_NeckAngle();
  virtual void Emergency();


  virtual void callbackThread();
  // virtual void Emergencycallback(const std_msgs::Bool &msg);
  ros::Publisher joint_state_publisher_; ///< Publishes joint states from reads
  ros::Subscriber joint_state_subscriber_; ///< Gets joint states for writes
  ros::Subscriber FSR_L_sensor_subscriber_; ///< Gets FSR Sensor data from Arduino FSR_L
  ros::Subscriber FSR_R_sensor_subscriber_; ///< Gets FSR Sensor data from Arduino FSR_R
  ros::Subscriber IMU_sensor_subscriber_; ///< Gets IMU Sensor data from XSENSE mti_driver_node
  // ros::Subscriber Emergency_subscriber_; ///< Emergency Subscribe


  // Variable
  const int SPIN_RATE;
  VectorXd Goal_joint_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
  uint8_t L_value = 0;
  uint8_t R_value = 0;
  uint8_t fsr_value[2] = {L_value, R_value};
  VectorXd quaternion = VectorXd::Zero(4);
  VectorXd RPY = VectorXd::Zero(3);   // Roll Pitch Yaw
  VectorXd Accel = VectorXd::Zero(3); // Accel_x, Accel_y, Accel_z
  VectorXd Gyro = VectorXd::Zero(3);  // Gyro_x, Gyro_y, Gyro_z
  double rl_neckangle = 0;
  double ud_neckangle = 0;
  bool emergency_ = 1; // True : Keep going , False : Emergency


  int8_t mode = 0;
  double walkfreq = 1.48114;
  double walktime = 2 / walkfreq;
  int freq = 100;
  int indext = 0;
  int stop_indext = 0;
  int emergency = 0;
  MatrixXd RL_motion;
  MatrixXd LL_motion;
  MatrixXd RL_motion0;
  MatrixXd LL_motion0;
  MatrixXd RL_motion1;
  MatrixXd LL_motion1;
  MatrixXd RL_motion2;
  MatrixXd LL_motion2;
  MatrixXd RL_motion3;
  MatrixXd LL_motion3;
  MatrixXd RL_motion4;
  MatrixXd LL_motion4;
  MatrixXd RL_motion5;
  MatrixXd LL_motion5;
  MatrixXd RL_motion6;
  MatrixXd LL_motion6;
  MatrixXd RL_motion7;
  MatrixXd LL_motion7;
  VectorXd All_Theta = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);

  // tf2::Quaternion quaternion;
};

#endif // CALLBACK_H