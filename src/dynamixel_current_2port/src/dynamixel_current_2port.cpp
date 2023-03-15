#include "dynamixel_current_2port.hpp"

//생성자
Dxl::Dxl()
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort())
        ROS_ERROR("Failed to open the port!");
    if (!portHandler->setBaudRate(BAUDRATE))
        ROS_ERROR("Failed to set the baudrate!");

    //Current Control Mode 
    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_OperatingMode, Current_Control_Mode, &dxl_error); // current_mode
        if (dxl_comm_result != COMM_SUCCESS)
            ROS_ERROR("Failed to set torque control mode for Dynamixel ID %d", i);
    }

    // for(uint8_t i=0;i<4;i++) packetHandler->write1ByteTxRx(portHandler, dx_id[i], DxlReg_OperatingMode, Position_Control_Mode, &dxl_error);   // position_mode

    //Torque Enable
    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_TorqueEnable, 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
            ROS_ERROR("Failed to enable torque for Dynamixel ID %d", i);
    }

    //LED ON
    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_LED, 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
            ROS_ERROR("Failed to enable LED for Dynamixel ID %d", i);
    }
}

//소멸자
Dxl::~Dxl()
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_TorqueEnable, Current_Control_Mode, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
            ROS_ERROR("Failed to disable torque for Dynamixel ID %d", i);
    }

    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_LED, 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
            ROS_ERROR("Failed to disable LED for Dynamixel ID %d", i);
    }

    portHandler->closePort();
}

//dxl 초기 세팅
void Dxl::initActuatorValues()
{
    torque2value[0] = TORQUE_TO_VALUE_MX_64;
    torque2value[1] = TORQUE_TO_VALUE_MX_106;
    torque2value[2] = TORQUE_TO_VALUE_MX_106;
    torque2value[3] = TORQUE_TO_VALUE_MX_106;
    torque2value[4] = TORQUE_TO_VALUE_MX_106;
    torque2value[5] = TORQUE_TO_VALUE_MX_106;

    zero_manual_offset[0] = 0;
    zero_manual_offset[1] = 0;
    zero_manual_offset[2] = 0;
    zero_manual_offset[3] = 0;
    zero_manual_offset[4] = 0;
    zero_manual_offset[5] = 0;
}

//각도 읽기(rad)
void Dxl::syncReadTheta()
{
    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, DxlReg_PresentPosition, 4);
    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) groupSyncRead.addParam(dxl_id[i]);
    groupSyncRead.txRxPacket();
    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) position[i] = groupSyncRead.getData(dxl_id[i], DxlReg_PresentPosition, 4);
    groupSyncRead.clearParam();
    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) th_[i] = convertValue2Radian(position[i]) - PI - zero_manual_offset[i];
}

//Value2Radian
float Dxl::convertValue2Radian(int32_t value)
{
    float radian = value / RAD_TO_VALUE;
    return radian;
}




int main(int argc, char **argv)
{

    ros::init(argc, argv, "daynmixel_current_2port");
    ros::NodeHandle nh;

    ROS_INFO("daynmixel_current_2port!");
}