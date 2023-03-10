#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include "std_msgs/String.h"
#include "dynamixel_sdk/dynamixel_sdk.h"


//Protocol version
#define PROTOCOL_VERSION         2.0

//Default setting
#define NUMBER_OF_DYNAMIXELS     6
#define BAUDRATE                 4500000 
#define GOAL_CURRENT_VALUE       1000
#define DEVICE_NAME              "/dev/ttyACM0"

#define PI                       3.141592
using Eigen::VectorXd;

// Operating Mode
enum DynamixelOperatingMode
{
    Current_Control_Mode = 0,
    Velocity_Control_Mode = 1,
    Position_Control_Mode = 3,
    Extended_Position_Control_Mode = 4,
    Current_based_Position_Control_Mode = 5,
    PWM_Control_Mode = 16
} DynamixelOperatingMode;

// Control table address
enum DynamixelStandardRegisterTable
{
  // EEPROM
  DxlReg_ModelNumber = 0,
  DxlReg_ModelInfo = 2,
  DxlReg_FirmwareVersion = 6,
  DxlReg_ID = 7,
  DxlReg_BaudRate = 8,
  DxlReg_ReturnDelayTime = 9,
  DxlReg_DriveMode = 10,        // !
  DxlReg_OperatingMode = 11,
  DxlReg_ShadowID = 12,
  DxlReg_ProtocolVersion = 13,
  DxlReg_HomingOffset = 20,
  DxlReg_MovingThreshold = 24,
  DxlReg_TemperatureLimit = 31,
  DxlReg_MaxVoltageLimit = 32,
  DxlReg_MinVoltageLimit = 34,
  DxlReg_PWMLimit = 36,
  DxlReg_CurrentLimit = 38,
  DxlReg_AccelerationLimit = 40,
  DxlReg_VelocityLimit = 44,
  DxlReg_MaxPositionLimit = 48,
  DxlReg_MinPositionLimit = 52, 
  DxlReg_DataPort1Mode = 56,
  DxlReg_DataPort2Mode = 57,
  DxlReg_DataPort3Mode = 58,
  DxlReg_Shutdown = 63,

  // RAM
  DxlReg_TorqueEnable = 64,
  DxlReg_LED = 65,
  DxlReg_StatusReturnLevel = 68,
  DxlReg_RegisteredInstruction = 69,
  DxlReg_HardwareErrorStatus = 70,
  DxlReg_VelocityIGain = 76,
  DxlReg_VelocityPGain = 78,
  DxlReg_PositionDGain = 80,
  DxlReg_PositionIGain = 82,
  DxlReg_PositionPGain = 84,
  DxlReg_Feedforward2ndGain = 88,
  DxlReg_Feedforward1stGain = 90,
  DxlReg_BusWatchdog = 98,
  DxlReg_GoalPWM = 100,
  DxlReg_GoalCurrent = 102,
  DxlReg_GoalVelocity = 104,
  DxlReg_ProfileAcceleration = 108,
  DxlReg_ProfileVelocity = 112,
  DxlReg_GoalPosition = 116,
  DxlReg_RealtimeTick = 120,
  DxlReg_Moving = 122,
  DxlReg_MovingStatus = 123,
  DxlReg_PresentPWM = 124,
  DxlReg_PresentCurrent = 126,
  DxlReg_PresentVelocity = 128,
  DxlReg_PresentPosition = 132,
  DxlReg_VelocityTrajectory = 136,
  DxlReg_PositionTrajectory = 140,
  DxlReg_PresentInputVoltage = 144,
  DxlReg_PresentTemperature = 146,
  DxlReg_DataPort1 = 152,
  DxlReg_DataPort2 = 154,
  DxlReg_DataPort3 = 156,
  DxlReg_IndirectAddress1 = 168,
  DxlReg_IndirectData1 = 224
} DynamixelStandardRegisterTable;


class Dxl
{
    //Member Variable
    private:
        dynamixel::PortHandler* portHandler;
        dynamixel::PacketHandler* packetHandler;

        const int dxl_id[NUMBER_OF_DYNAMIXELS] = { 0 };
        float zero_manual_offset[NUMBER_OF_DYNAMIXELS] = { 0 };
        uint32_t position[NUMBER_OF_DYNAMIXELS] = { 0 };
        uint32_t velocity[NUMBER_OF_DYNAMIXELS] = { 0 };
        int32_t ref_torque_value[NUMBER_OF_DYNAMIXELS] = { 0 };
        int32_t torque2value[NUMBER_OF_DYNAMIXELS] = { 0 };

        VectorXd ref_th_value = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        VectorXd ref_th_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        VectorXd ref_th_dot_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        VectorXd ref_torque_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        VectorXd th_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        VectorXd th_last_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        VectorXd th_dot_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        VectorXd th_dot_est_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        VectorXd tau_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
    //Member Function
        // virtual void syncReadTheta();
        // virtual void initActuatorValues();
        // virtual void syncReadThetaDot();
        // virtual void syncWriteTheta();
        // virtual void syncWriteTorque();
        // virtual void getParam(int32_t data, uint8_t *param);
        // float convertValue2Radian(int32_t value);
        // int32_t torqueToValue(double torque, uint8_t index);



    // Member Function
    public:
        Dxl(); //?????????
        ~Dxl(); //?????????

        // virtual void SetTorqueRef(VectorXd);
        // // virtual VectorXd GetTorqueAct();
        // virtual void SetThetaRef(VectorXd);
        // virtual VectorXd GetThetaAct();
        // virtual VectorXd GetThetaDot();
        // virtual VectorXd GetThetaDotEstimated();
        // virtual void Loop(bool RxTh, bool RxThDot, bool TxTorque);
        // virtual void CalculateEstimatedThetaDot(int);
};


#endif