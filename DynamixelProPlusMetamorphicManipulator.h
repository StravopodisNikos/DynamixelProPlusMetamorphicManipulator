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
//#include "definitions.h"                           // Includes definitions of control table variables addresses/data lengths/ communication/ protocols
//#include "motorIDs.h"                              // Includes motor IDs as set using Dynamixel Wizard

using namespace std;

//uint8_t dxl_id[] = {DXL1_ID, DXL2_ID, DXL3_ID};
extern uint8_t dxl_id[];
const int DXL_ID_SIZE = 3;                            // Must be configured based on declaration in test_metamorphic_manipulator.ino

extern int dxl_comm_result;                                                               // Communication result
extern bool dxl_addparam_result;                                                          // addParam result
extern bool dxl_getdata_result;                                                           // GetParam result
extern bool return_function_state;
extern uint8_t dxl_error; 
extern uint16_t dxl_model_number[];
extern uint32_t dxl_present_position[];

extern uint8_t dxl_ledBLUE_value[];
extern uint8_t dxl_ledGREEN_value[];
extern uint8_t dxl_ledRED_value[];

extern int32_t position_in_dxl_pulses;
extern double position_in_radians;

/*
typedef int32_t dxlVelLimit[sizeof(dxl_id)];
typedef int32_t dxlAccelLimit[sizeof(dxl_id)];
typedef int32_t dxlProfVel[sizeof(dxl_id)];
typedef int32_t dxlProfAccel[sizeof(dxl_id)];
typedef int32_t dxlGoalPos[sizeof(dxl_id)];
typedef int32_t typeDxlTrapzProfParams[sizeof(dxl_id)];
typedef uint8_t typeLED[sizeof(dxl_id)];
*/

extern int32_t dxl_vel_limit[DXL_ID_SIZE];
extern int32_t dxl_accel_limit[DXL_ID_SIZE];
extern int32_t dxl_prof_vel[DXL_ID_SIZE];
extern int32_t dxl_prof_accel[DXL_ID_SIZE];

typedef int32_t dxlGoalPos[DXL_ID_SIZE];
typedef int32_t dxlVelLimit[DXL_ID_SIZE];
typedef int32_t dxlAccelLimit[DXL_ID_SIZE];
typedef int32_t typeDxlTrapzProfParams[DXL_ID_SIZE];
typedef uint8_t typeLED[DXL_ID_SIZE];

// Function Nomenclature

/*
 *      Functions with syncSet{Name}: Use GroupSyncWrite with Direct Addressing of only 1 control table item
 *      Functions with syncSet_{Name 1}_{Name 2}_{Name n}: Use GroupSyncWrite with Indirect Addressing of n control table items
 *      
 *      Functions with syncGet{Name}: Use GroupSyncRead with Direct Addressing of only 1 control table item
 *      Functions with syncGet_{Name 1}_{Name 2}_{Name n}: Use GroupSyncRead with Indirect Addressing of n control table items
 */

class DynamixelProPlusMetamorphicManipulator
{ 
 public:
    int led_change = 0;                         // global value to see led colours changing after each movement completes(just for simple visualization)

    DynamixelProPlusMetamorphicManipulator();

    bool pingDynamixels(uint8_t *DxlIDs, int DxlIds_size, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler);

    bool syncSetTorque(uint8_t *DxlIDs, int DxlIds_size, uint8_t data, dynamixel::GroupSyncWrite groupSyncWrite_TORQUE_ENABLE, dynamixel::PacketHandler *packetHandler);

    bool syncSetVelAccelLimit(uint8_t *DxlIDs, int DxlIds_size, dxlVelLimit dxl_vel_limit, int dxl_vel_limit_size, dxlAccelLimit dxl_accel_limit, int dxl_accel_limit_size, dynamixel::GroupSyncWrite groupSyncWriteVelLim, dynamixel::GroupSyncWrite groupSyncWriteAccelLim, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler);

    bool syncSetGoalPosition(uint8_t *DxlIDs, int DxlIds_size, dxlGoalPos dxl_goal_pos, int dxl_goal_pos_size, dynamixel::GroupSyncWrite groupSyncWriteGoalPos,  dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler);

    bool syncSet_GP_A_V_LED( uint8_t *DxlIDs, int DxlIds_size, typeDxlTrapzProfParams DxlTrapzProfParams[], int DxlTrapzProfParams_size, dynamixel::GroupSyncWrite groupSyncWrite_GP_A_V_LED, dynamixel::GroupSyncWrite groupSyncWrite_TORQUE_ENABLE, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler);

    bool syncGet_PP_MV( uint8_t *DxlIDs, int DxlIds_size, uint8_t *dxl_moving, int dxl_moving_size, uint32_t *dxl_present_position, int dxl_present_position_size , dynamixel::GroupSyncRead groupSyncRead_PP_MV, dynamixel::GroupSyncWrite groupSyncWrite_TORQUE_ENABLE, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler);

    bool syncGet_PP_PV_PA_VL_AL( uint8_t *DxlIDs, int DxlIds_size, uint32_t *dxl_present_position, int dxl_present_position_size, int32_t dxl_prof_vel[], int dxl_prof_vel_size, int32_t dxl_prof_accel[], int dxl_prof_accel_size , int32_t dxl_vel_limit[], int dxl_vel_limit_size,  int32_t dxl_accel_limit[], int dxl_accel_limit_size, dynamixel::GroupSyncRead groupSyncRead_PP_PV_PA_VL_AL, dynamixel::GroupSyncWrite groupSyncWrite_TORQUE_ENABLE, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler);
    
    int32_t convertRadian2DxlPulses(double position_in_radians);

    double convertDxlPulses2Radian(int32_t position_in_dxl_pulses);

    private:


};

 #endif