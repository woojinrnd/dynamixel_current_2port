#include "callback.hpp"

extern Dxl _dxl;

Callback::Callback() {}

sensor_msgs::JointState joint_state;

void Callback::JointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    for (int i=0; i<NUMBER_OF_DYNAMIXELS;i++) 
        Goal_joint_[i] = msg->position[i];
}
