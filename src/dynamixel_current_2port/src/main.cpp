#include <iostream>
#include "dynamixel.hpp"
#include "dynamixel_controller.hpp"

// Dxl dxl;
Dxl_Controller dxl_ctrl;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "daynmixel_current_2port");
    ros::Time::init();
    ros::Rate loop_rate(300);
    ros::NodeHandle nh;

    while(ros::ok())
    {
    //     VectorXd A(6);
    //     A << 10, 10, 10, 10, 10, 10;
    //     dxl.SetThetaRef(A);
        // dxl.SetPresentMode(0);
        // dxl.SetThetaRef(A);
    }
    ROS_INFO("daynmixel_current_2port!");

}