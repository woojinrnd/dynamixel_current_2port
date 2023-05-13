#include "callback.hpp"

extern Dxl dxl;

Callback::Callback() {}

sensor_msgs::JointState joint_state;

void Callback::JointStatesCallback(const sensor_msgs::JointState::ConstPtr &joint_command)
{
    for (int i=0; i<NUMBER_OF_DYNAMIXELS;i++)
    {
        Goal_joint_[i] = joint_command->position[i];
        dxl.SetThetaRef(Goal_joint_);
    } 
}


void Callback::sensorCallback(const std_msgs::Int32ConstPtr &FSR)
{
    fsr_value = FSR->data;
}



