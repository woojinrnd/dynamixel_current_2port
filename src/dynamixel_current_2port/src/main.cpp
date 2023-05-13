#include <iostream>
#include "dynamixel.hpp"
#include "callback.hpp"
#include "dynamixel_controller.hpp"

Dxl dxl;
Callback callback;
Dxl_Controller dxl_ctrl;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "daynmixel_current_2port");
    ros::Time::init();
    ros::Rate loop_rate(300);
    ros::NodeHandle nh;


    ros::Publisher joint_state_publisher_;   ///< Publishes joint states from reads
    joint_state_publisher_ = nh.advertise<sensor_msgs::JointState>("KWJ_joint_states", 100);
    
    ros::Subscriber joint_state_subscriber_; ///< Gets joint states for writes
    joint_state_subscriber_ = nh.subscribe("KWJ_desired_joint_states", 1000, &Callback::JointStatesCallback, &callback);

    ros::Subscriber FSR_sensor_subscriber_; ///< Gets FSR Sensor data from Arduino FSR 
    FSR_sensor_subscriber_ = nh.subscribe("FSR", 1000, &Callback::sensorCallback, &callback);

    VectorXd A(6);
    VectorXd B(6);

    dxl.initActuatorValues();

    for (int i = 0; i < 6; i++)
    {
        A[i] = 0;
    }

    // dxl.SetThetaRef(A);
    // dxl.syncWriteTheta();
    

    while (ros::ok())
    {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();

        std::vector<std::string> joint_name = {"j1", "j2", "j3", "j4", "j5", "j6"};
    

        for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
        {
            msg.name.push_back(joint_name.at(i));
            dxl.GetThetaAct();
            msg.position.push_back(dxl.th_[i]);
        }
        
        joint_state_publisher_.publish(msg);
        dxl.FSR_flag();  
        dxl.syncWriteTheta();
        std::cout << callback.fsr_value << std::endl;
        

        // ROS_INFO("Position : %d", dxl.syncReadTheta());
        // std::cout << dxl.GetThetaAct() << std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }

    // ROS_INFO("daynmixel_current_2port!");
    // dxl.~Dxl();
    return 0;
}
