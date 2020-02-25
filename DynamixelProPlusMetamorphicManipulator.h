 /*
  * DynamixelProPlusMetamorphicManipulator.cpp  - Library for controlling Dynamixels Pro+ of a Metamorphic Manipulator
  * Created by N.A. Stravopodis, February, 2020.
  */

#ifndef DynamixelProPlusMetamorphicManipulator_h
#define DynamixelProPlusMetamorphicManipulator_h

#include "Arduino.h"
#include <DynamixelSDK.h>
#include "OpenCR.h"
#include <vector>
#include <fstream>

using namespace std;

extern int dxl_comm_result;                                                        // Communication result
extern bool dxl_addparam_result;                                                          // addParam result
extern bool dxl_getdata_result;                                                           // GetParam result
extern uint8_t dxl_error; 
extern uint16_t dxl_model_number[];
extern int32_t dxl_present_position[];

typedef int32_t dxlVelLimit[3];
typedef int32_t dxlAccelLimit[3];
typedef int32_t dxlGoalPos[3];

class DynamixelProPlusMetamorphicManipulator
{ 
 public:

    DynamixelProPlusMetamorphicManipulator();

    bool pingDynamixels(uint8_t *DxlIDs, int DxlIds_size, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler);

    bool syncSetTorque(uint8_t *DxlIDs, int DxlIds_size, uint8_t data, dynamixel::GroupSyncWrite groupSyncWrite_TORQUE_STATUS, dynamixel::PacketHandler *packetHandler);

    bool syncSetVelAccelLimit(uint8_t *DxlIDs, int DxlIds_size, dxlVelLimit dxl_vel_limit, int dxl_vel_limit_size, dxlAccelLimit dxl_accel_limit, int dxl_accel_limit_size, dynamixel::GroupSyncWrite groupSyncWriteVelLim, dynamixel::GroupSyncWrite groupSyncWriteAccelLim, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler);

    bool syncSetGoalPosition(uint8_t *DxlIDs, int DxlIds_size, dxlGoalPos dxl_goal_pos, int dxl_goal_pos_size, dynamixel::GroupSyncWrite groupSyncWriteGoalPos,  dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler);

    private:

};

 #endif