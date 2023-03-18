#include "dynamixel_current_2port.hpp"

//Constructor
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

//Destructor
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

// ************************************ GETTERS ***************************************** //

//Getter() : 각도 읽기(raw->rad)
void Dxl::syncReadTheta()
{
    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, DxlReg_PresentPosition, 4);
    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) groupSyncRead.addParam(dxl_id[i]);
    groupSyncRead.txRxPacket();
    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) position[i] = groupSyncRead.getData(dxl_id[i], DxlReg_PresentPosition, 4);
    groupSyncRead.clearParam();
    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) th_[i] = convertValue2Radian(position[i]) - PI - zero_manual_offset[i];
}

//Getter() : velocity 읽기 (raw data)
void Dxl::syncReadThetaDot()
{
    dynamixel::GroupSyncRead groupSyncReadThDot(portHandler, packetHandler, DxlReg_PresentVelocity, 4);
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++) groupSyncReadThDot.addParam(dxl_id[i]);
    groupSyncReadThDot.txRxPacket();
    for(uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++) velocity[i] = groupSyncReadThDot.getData(dxl_id[i], DxlReg_PresentVelocity, 4);
    groupSyncReadThDot.clearParam();
}

//Getter() : 각도 getter()
VectorXd Dxl::GetThetaAct()
{
    return th_;
}

//Getter() : About dynamixel packet data
void Dxl::getParam(int32_t data, uint8_t *param)
{
  param[0] = DXL_LOBYTE(DXL_LOWORD(data));
  param[1] = DXL_HIBYTE(DXL_LOWORD(data));
  param[2] = DXL_LOBYTE(DXL_HIWORD(data));
  param[3] = DXL_HIBYTE(DXL_HIWORD(data));
}

//Getter() : 추정계산 (이전 세타값 - 현재 세타값 / 시간)
void Dxl::CalculateEstimatedThetaDot(int dt_us)
{
    th_dot_est_ = (th_last_ - th_) / (-dt_us * 0.00001);
    th_last_ = th_;
}

//Getter() : 각속도 추정계산 getter() 
VectorXd Dxl::GetThetaDotEstimated()
{
    return th_dot_est_;
}

//Getter() : 각속도 getter()
VectorXd Dxl::GetThetaDot()
{
    VectorXd vel_(NUMBER_OF_DYNAMIXELS);
    for(uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++)
    {
        if(velocity[i] > 4294900000) vel_[i] = (velocity[i] - 4294967295) * 0.003816667; //4,294,967,295 = 0xFFFFFFFF   // 1 = 0.229rpm   // 1 = 0.003816667
        else vel_[i] = velocity[i] * 0.003816667;
    }
    return vel_;
}

// **************************** SETTERS ******************************** //

//setter() : 각도 setter()
void Dxl::syncWriteTheta()
{
    dynamixel::GroupSyncWrite SyncWriteTh(portHandler, packetHandler, DxlReg_GoalPosition, 4);
    uint8_t parameter[NUMBER_OF_DYNAMIXELS] = {0, 0, 0, 0, 0, 0};

    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++)
    {
        ref_th_value = ref_th_ * RAD_TO_VALUE;
        getParam(ref_th_value[i], parameter);
        SyncWriteTh.addParam(dxl_id[i], (uint8_t *)&parameter);
    }
    SyncWriteTh.txPacket();
    SyncWriteTh.clearParam();
}

//setter() : 토크 setter()
void Dxl::syncWriteTorque()
{
    dynamixel::GroupSyncWrite groupSyncWriter(portHandler, packetHandler, DxlReg_GoalCurrent, 2);
    uint8_t parameter[NUMBER_OF_DYNAMIXELS] = {0, 0, 0, 0, 0, 0};
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++)
    {
        ref_torque_value[i] = torqueToValue(ref_torque_[i], i);
        if(ref_torque_value[i] > 1000) ref_torque_value[i] = 1000; //상한값
        else if(ref_torque_value[i] < -1000) ref_torque_value[i] = -1000; //하한값
    }
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++)
    {
        getParam(ref_torque_value[i], parameter);
        groupSyncWriter.addParam(dxl_id[i], (uint8_t *)&parameter);
    }
    groupSyncWriter.txPacket();
    groupSyncWriter.clearParam();
}

//Setter() : 목표 세타값 설정
void Dxl::SetThetaRef(VectorXd theta)
{
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS;i++) ref_th_[i] = theta[i];
}

//Setter() : 목표 토크 설정
void Dxl::SetTorqueRef(VectorXd a_torque)
{
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++) ref_torque_[i] = a_torque[i];
}

// **************************** Function ******************************** //

//Torque2Value : 토크 -> 로우 data
int32_t Dxl::torqueToValue(double torque, uint8_t index)
{
    int32_t value_ = int(torque * torque2value[index]); //MX-64
    return value_;
}

//Value2Radian (Raw data -> Radian)
float Dxl::convertValue2Radian(int32_t value)
{
    float radian = value / RAD_TO_VALUE;
    return radian;
}

//각도(rad), 각속도(rad/s) 읽고, torque(Nm->raw) 쓰기
void Dxl::Loop(bool RxTh, bool RxThDot, bool TxTorque)
{
    if(RxTh) syncReadTheta();
    if(RxThDot) syncReadThetaDot();
    if(TxTorque) syncWriteTorque();
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

