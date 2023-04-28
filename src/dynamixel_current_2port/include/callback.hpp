#ifndef CALLBACK_H
#define CALLBACK_H

#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "dynamixel.hpp"

using Eigen::VectorXd;


class Callback
{
public:
  Callback();

  //Function
  virtual void JointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  
  //Variable
  VectorXd Goal_joint_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
  
};

#endif // CALLBACK_H