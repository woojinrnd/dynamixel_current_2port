#include "dynamixel_current_2port.hpp"


// PortHandler* portHandler = PortHandler::getPortHandler(DEVICE_NAME);
// PacketHandler* packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);


// GroupSyncRead groupSyncRead(portHandler, packetHandler, DxlReg_PresentCurrent, 2);
// GroupSyncWrite groupSyncWrite(portHandler, packetHandler, DxlReg_GoalCurrent, 2);










int main(int argc, char **argv)
{

    ros::init(argc, argv, "daynmixel_current_2port");
    ros::NodeHandle nh;

    ROS_INFO("daynmixel_current_2port!");
}