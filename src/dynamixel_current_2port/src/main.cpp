#include <iostream>
#include "dynamixel.hpp"
#include "callback.hpp"
#include "dynamixel_controller.hpp"
#include "Walkingpattern_generator.hpp"

Dxl dxl;
Callback callback;
Dxl_Controller dxl_ctrl;
Motions motion;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "daynmixel_current_2port");
    ros::Time::init();
    ros::Rate loop_rate(300);
    ros::NodeHandle nh;
    // ros::AsyncSpinner spinner(0); // Multi-threaded spinning
    // spinner.start(); // Multi-threaded spinning


    ros::Publisher joint_state_publisher_;   ///< Publishes joint states from reads
    joint_state_publisher_ = nh.advertise<sensor_msgs::JointState>("KWJ_joint_states", 100);
    
    ros::Subscriber joint_state_subscriber_; ///< Gets joint states for writes
    joint_state_subscriber_ = nh.subscribe("KWJ_desired_joint_states", 1000, &Callback::JointStatesCallback, &callback);

    ros::Subscriber FSR_sensor_subscriber_; ///< Gets FSR Sensor data from Arduino FSR 
    FSR_sensor_subscriber_ = nh.subscribe("FSR", 1000, &Callback::sensorCallback, &callback);

    // ros::waitForShutdown(); // Multi-threaded spinning

    VectorXd A(6);

    dxl.initActuatorValues();

    for (int i=0; i<6;i++)
    {
        A[i] = 0;
    }
    dxl.SetThetaRef(A);
    dxl.syncWriteTheta();

    //About motion
    // motion.Motion1();
    // MatrixXd RL_motion1 = motion.Return_Motion1_RL();
    // std::cout << RL_motion1 << std::endl;
    // int t=0;
    // int indext =0;

    while (ros::ok())
    {
        //About motion
        // t += 1;
        // if (t == 10)
        // {indext +=1;}

        // for (int i = 0; i < 6; i++)
        // {
        //     A[i] = RL_motion1(indext, i);
        // }
        // std::cout << A << std::endl;

        // if (indext >= 923)
        //     indext = 0;
        // dxl.SetThetaRef(A);

        dxl.syncWriteTheta();

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
        // std::cout << callback.fsr_value << std::endl;

        ros::spinOnce();
        loop_rate.sleep();
    }

    // ROS_INFO("daynmixel_current_2port!");
    // dxl.~Dxl();
    return 0;
}
