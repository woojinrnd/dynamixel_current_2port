// // #include <iostream>
// // #include <ros/ros.h>
// // #include "dynamixel_sdk/dynamixel_sdk.h"
// #include "dynamixel.hpp"
// // #include "dynamixel_controller.hpp"

// // Dxl dxl;
// // Dxl_Controller dxl_ctrl;

// dynamixel::PortHandler *portHandler;
// dynamixel::PacketHandler *packetHandler;
// const int ID[6] = {0, 3, 4, 6, 9, 11};

// int baudrate = 4000000;
// int goal_current_value[6] = {10, 0, 10, 0, 10, 0};
// int present_current_value[6] = {0};

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "daynmixel_current_2port");
//     ros::Time::init();
//     ros::Rate loop_rate(1);
//     ros::NodeHandle nh;

//     portHandler = dynamixel::PortHandler::getPortHandler("/dev/ttyACM0");
//     packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);
//     if (portHandler->openPort())
//     {
//         ROS_INFO("Succeeded to open the port!");
//     }
//     else
//     {
//         ROS_ERROR("Failed to open the port!");
//         return 0;
//     }

//     portHandler->setBaudRate(baudrate);

//     // Set the current control mode for the Dynamixel actuator
//     uint8_t dxl_error = 0;
//     for (int i = 0; i < 6; i++)
//     {
//         int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, ID[i], DxlReg_OperatingMode, Current_Control_Mode, &dxl_error); // current_mode

//         dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, ID[i], 64, 1, &dxl_error);
//         if (dxl_comm_result != COMM_SUCCESS)
//         {
//             ROS_ERROR("%s", packetHandler->getTxRxResult(dxl_comm_result));
//             return 0;
//         }
//         else if (dxl_error != 0)
//         {
//             ROS_ERROR("%s", packetHandler->getRxPacketError(dxl_error));
//             return 0;
//         }
//         else
//         {
//             ROS_INFO("Successfully set the current control mode!");
//         }
//     }

//     while (ros::ok())
//     {
//         // dxl_ctrl.GetJointTheta();
//         // VectorXd A(6);
//         // A << 10, 10, 10, 10, 10, 10;
//         // dxl.SetThetaRef(A);
//         // dxl.syncWriteTheta();
//         // dxl.writeTheta();
//         // std::cout << dxl.GetThetaAct() << std::endl;
//         // dxl.SetPresentMode(0);
//         // dxl.SetThetaRef(A);
//         for (int i = 0; i < 6; i++)
//         {
//             int dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, ID[i], 132, (uint32_t *)&present_current_value, &dxl_error);
//             if (dxl_comm_result != COMM_SUCCESS)
//             {
//                 ROS_ERROR("%s", packetHandler->getTxRxResult(dxl_comm_result));
//                 return 0;
//             }
//             else if (dxl_error != 0)
//             {
//                 ROS_ERROR("%s", packetHandler->getRxPacketError(dxl_error));
//                 return 0;
//             }
//             else
//             {
//                 // ROS_INFO("Successfully wrote the current value to the Dynamixel actuator!");
//                 // current_pub.publish(present_current_value);
//                 ROS_INFO("Present pos value : %d", present_current_value);
//             }

//             dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, ID[i], 116, goal_current_value[i], &dxl_error);
//             if (dxl_comm_result != COMM_SUCCESS)
//             {
//                 ROS_ERROR("%s", packetHandler->getTxRxResult(dxl_comm_result));
//                 return 0;
//             }
//             else if (dxl_error != 0)
//             {
//                 ROS_ERROR("%s", packetHandler->getRxPacketError(dxl_error));
//                 return 0;
//             }
//             else
//             {
//                 // ROS_INFO("Successfully wrote the current value to the Dynamixel actuator!");
//                 ROS_INFO("Goal Current value : %d",goal_current_value);
//             }
//         }
//     }
//     ROS_INFO("daynmixel_current_2port!");
// }



#include <iostream>
#include "dynamixel.hpp"
// #include "dynamixel_controller.hpp"

Dxl dxl;
// Dxl_Controller dxl_ctrl;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "daynmixel_current_2port");
    ros::Time::init();
    ros::Rate loop_rate(300);
    ros::NodeHandle nh;

    // // dxl_ctrl.GetJointTheta();

    VectorXd A(6);
    VectorXd B(1);
    A << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
    // A << 0, 0, 0, 0, 0, 0;
    // B << 1;
    dxl.SetThetaRef(A);
    dxl.syncWriteTheta();
    std::cout << dxl.GetThetaAct() << std::endl;
    

    // dxl.writeTheta();
    // dxl.writeTheta();
    // std::cout << dxl.GetThetaAct() << std::endl;

    
    while(ros::ok())
    {
        // dxl.syncWriteTheta();
    }
    // if (dxl.GetThetaAct() == A) return 0;
    // else if (dxl.GetThetaAct() == B) return 0;

    ROS_INFO("daynmixel_current_2port!");

    // dxl.~Dxl();
    // return 0;
}
