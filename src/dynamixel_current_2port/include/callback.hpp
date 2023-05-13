#ifndef CALLBACK_H
#define CALLBACK_H

#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "dynamixel.hpp"

using Eigen::VectorXd;


class Callback
{
public:
  Callback();

  //Function
  virtual void JointStatesCallback(const sensor_msgs::JointState::ConstPtr &joint_command);
  virtual void sensorCallback(const std_msgs::Int32ConstPtr &FSR);
  //Variable
  VectorXd Goal_joint_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
  int32_t fsr_value = 0;
  
};

#endif // CALLBACK_H