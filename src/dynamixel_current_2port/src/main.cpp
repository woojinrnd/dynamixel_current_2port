#include <iostream>
#include "dynamixel.hpp"
#include "callback.hpp"
#include "dynamixel_controller.hpp"

Dxl dxl;
Callback callback;
// Dxl_Controller dxl_ctrl;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "daynmixel_current_2port");
    ros::Time::init();
    ros::Rate loop_rate(300);
    ros::NodeHandle nh;
    

    ros::Publisher joint_state_publisher_;   ///< Publishes joint states from reads
    joint_state_publisher_ = nh.advertise<sensor_msgs::JointState>("joint_states", 100);
    
    ros::Subscriber joint_state_subscriber_; ///< Gets joint states for writes
    joint_state_subscriber_ = nh.subscribe("desired_joint_states", 10, &Callback::JointStatesCallback, &callback);

    // sensor_msgs::JointState joint_state;

    // joint_state.name.resize(NUMBER_OF_DYNAMIXELS);
    // joint_state.position.resize(NUMBER_OF_DYNAMIXELS);
    // joint_state.velocity.resize(NUMBER_OF_DYNAMIXELS);
    // joint_state.effort.resize(NUMBER_OF_DYNAMIXELS);

    // joint_state.name.push_back("joint_1, joint_2, joint_3, joint_4, joint_5, joint_6");
    // joint_state_publisher_.publish(joint_state);
    
    VectorXd A(6);
    

    dxl.SetJointName();
    
    

    dxl.initActuatorValues();

    // for (int i = 0; i < 6; i++)
    // {
    //     A[i] = 0;
    // }

    dxl.SetThetaRef(A);


    while (ros::ok())
    {
        dxl.syncWriteTheta();

    sensor_msgs::JointState joint_state;
    joint_state.name.push_back("joint_1, joint_2, joint_3, joint_4, joint_5, joint_6");
    joint_state.position.resize(NUMBER_OF_DYNAMIXELS);
    joint_state.velocity.resize(NUMBER_OF_DYNAMIXELS);
    joint_state.effort.resize(NUMBER_OF_DYNAMIXELS);

    for (int i = 0; i < NUMBER_OF_DYNAMIXELS; i++) A[i] = callback.Goal_joint_[i];
    dxl.SetThetaRef(A);
    joint_state_publisher_.publish(joint_state);
        // std::cout << dxl.GetThetaAct() << std::endl;
    }
    // if (dxl.GetThetaAct() == A) return 0;
    // else if (dxl.GetThetaAct() == B) return 0;

    ROS_INFO("daynmixel_current_2port!");

    // dxl.~Dxl();
    // return 0;
}
