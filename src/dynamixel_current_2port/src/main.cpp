#include "dynamixel.hpp"

Dxl dxl;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "daynmixel_current_2port");
    ros::Time::init();
    ros::Rate loop_rate(300);
    ros::NodeHandle nh;

    

    ROS_INFO("daynmixel_current_2port!");
}