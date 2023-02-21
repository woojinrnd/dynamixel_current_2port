#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include <ros/ros.h>

#include "std_msgs/String.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

//Protocol version
#define PROTOCOL_VERSION         2.0

//Default setting
#define NUMBER_OF_DYNAMIXELS     6
#define BAUDRATE                 4500000 
#define GOAL_CURRENT_VALUE       1000
#define DEVICE_NAME              "/dev/ttyACM0"


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
  DxlReg_IndirectData1 = 224,
};

dynamixel::GroupSyncRead::

class Dxl
{
    //Member Variable
    private:
        dynamixel::PortHandler* portHandler;
        dynamixel::PacketHandler* packetHandler;
      
    //Member Function
    public:
        //기본 생성자
        Dxl()
        {
            const int dxl_id[NUMBER_OF_DYNAMIXELS] = { 0 };
            float zero_manual_offset[NUMBER_OF_DYNAMIXELS] = { 0 };
            uint32_t position[NUMBER_OF_DYNAMIXELS] = { 0 };
            uint32_t velocity[NUMBER_OF_DYNAMIXELS] = { 0 };
            int32_t 
        }
    
        

};


#endif