 /*
  * DynamixelProPlusMetamorphicManipulator.cpp  - Library for controlling Dynamixels Pro+ of a Metamorphic Manipulator
  * Created by N.A. Stravopodis, February, 2020.
  */

#include "Arduino.h"
#include <DynamixelSDK.h>
#include "OpenCR.h"
#include <fstream>
#include <vector>
#include "DynamixelProPlusMetamorphicManipulator.h"

using namespace std;

// Creating a ney type for storaging 2D arrays of parameter values for control tale items
typedef uint8_t paramAccelLimit[4];                                           
typedef uint8_t paramVelLimit[4];
typedef uint8_t paramGoalPos[4];

// Constructor
DynamixelProPlusMetamorphicManipulator::DynamixelProPlusMetamorphicManipulator(){
}

// =========================================================================================================== //

// Ping Dynamixels
bool DynamixelProPlusMetamorphicManipulator::pingDynamixels(uint8_t *DxlIDs, int DxlIds_size, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler) {

/*
 *  Pings Connected Dynamixels given the correct ID number as set using thw Wizard Software
 */
  for(int id_count = 0; id_count < DxlIds_size; id_count++){
      dxl_comm_result = packetHandler->ping(portHandler, DxlIDs[id_count], &dxl_model_number[id_count], &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
        return false;
      }
      else if (dxl_error != 0)
      {
        Serial.print(packetHandler->getRxPacketError(dxl_error));
        return false;
      }
    
      Serial.print("[ID:"); Serial.print(DxlIDs[id_count]);
      Serial.print("] ping Succeeded. Dynamixel model number : ");
      Serial.println(dxl_model_number[id_count]);
  }

  return true;
}

// =========================================================================================================== //

bool DynamixelProPlusMetamorphicManipulator::syncSetTorque(uint8_t *DxlIDs, int DxlIds_size, uint8_t data, dynamixel::GroupSyncWrite groupSyncWrite_TORQUE_STATUS, dynamixel::PacketHandler *packetHandler)
{     
/*
 *  Sets the same toque status to all connected Dynamixels
 * torque status value is given through data variable and range value is: 0~1 
 */
    for(int id_count = 0; id_count < DxlIds_size; id_count++){
        dxl_addparam_result = groupSyncWrite_TORQUE_STATUS.addParam(DxlIDs[id_count], &data);
        if (dxl_addparam_result != true)
        {
          Serial.print("[ID:"); Serial.print(DxlIDs[id_count]); Serial.println("groupSyncWrite_TORQUE_STATUS.addParam FAILED");
          return false;
        }
    }

    dxl_comm_result = groupSyncWrite_TORQUE_STATUS.txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(packetHandler->getTxRxResult(dxl_comm_result)); Serial.println("groupSyncWrite_TORQUE_STATUS.txPacket() FAILED");
      return false;
    }
    else
    {
      Serial.print("Torque STATUS CHANGED"); Serial.print(" New status:"); Serial.println(data);
    }
    
    groupSyncWrite_TORQUE_STATUS.clearParam();
    return true;
} // END FUNCTION 

// =========================================================================================================== //

bool DynamixelProPlusMetamorphicManipulator::syncSetVelAccelLimit(uint8_t *DxlIDs, int DxlIds_size, dxlVelLimit dxl_vel_limit, int dxl_vel_limit_size, dxlAccelLimit dxl_accel_limit, int dxl_accel_limit_size, dynamixel::GroupSyncWrite groupSyncWriteVelLim, dynamixel::GroupSyncWrite groupSyncWriteAccelLim, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler){
 
/*
 * Changes Limit Values for all Dynamixels. Torque MUST be DISABLED!
 * Different Limit value can be given to each Dynamixel
 * The number of limit values specified must equal the dynamixel motors!
 */

// Define 2D arrays for parameter storage
paramAccelLimit param_accel_limit[dxl_accel_limit_size];
paramVelLimit param_vel_limit[dxl_vel_limit_size];

// Checks if DxlIds_size==dxl_vel_limit_size==dxl_accel_limit_size
if ( (DxlIds_size == dxl_vel_limit_size) && (DxlIds_size == dxl_accel_limit_size) )
{
    // I. Set Limit values for Velocity to Dynamixels
    for(int id_count = 0; id_count < DxlIds_size; id_count++){
            // I.1. Allocate Acceleration Limit value into byte array
            param_accel_limit[id_count][0] = DXL_LOBYTE(DXL_LOWORD(dxl_accel_limit[id_count]));
            param_accel_limit[id_count][1] = DXL_HIBYTE(DXL_LOWORD(dxl_accel_limit[id_count]));
            param_accel_limit[id_count][2] = DXL_LOBYTE(DXL_HIWORD(dxl_accel_limit[id_count]));
            param_accel_limit[id_count][3] = DXL_HIBYTE(DXL_HIWORD(dxl_accel_limit[id_count]));
    }

for(int id_count = 0; id_count < DxlIds_size; id_count++){
    // I.2.Add Dynamixel#1 goal acceleration value to the Syncwrite storage
    dxl_addparam_result = groupSyncWriteAccelLim.addParam(DxlIDs[id_count], param_accel_limit[id_count]);
    if (dxl_addparam_result != true)
    {
      Serial.print("[ID:"); Serial.print(DxlIDs[id_count]); Serial.println("] groupSyncWriteAccelLim addparam failed");
      return false;
    }
    else
    {
      Serial.print("[ID:"); Serial.print(DxlIDs[id_count]); Serial.println("] groupSyncWriteAccelLim addparam successful");
    }
}

    // I.4.Syncwrite goal acceleration
    dxl_comm_result = groupSyncWriteAccelLim.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
       Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    }
    else
    {
      for(int id_count = 0; id_count < DxlIds_size; id_count++){  
        Serial.println("groupSyncWriteAccelLim txPacket successful");
        Serial.print("[ID:"); Serial.print(DxlIDs[id_count]);  Serial.print("] Acceleration Limit CHANGED"); Serial.print(" New Acceleration Limit:"); Serial.println(dxl_accel_limit[id_count]);
      }
    }
    
    // I.5.Clear syncwrite parameter storage
    groupSyncWriteAccelLim.clearParam();
    

    for(int id_count = 0; id_count < DxlIds_size; id_count++){
            // I.1. Allocate Velocity Limit value into byte array
            param_vel_limit[id_count][0] = DXL_LOBYTE(DXL_LOWORD(dxl_vel_limit[id_count]));
            param_vel_limit[id_count][1] = DXL_HIBYTE(DXL_LOWORD(dxl_vel_limit[id_count]));
            param_vel_limit[id_count][2] = DXL_LOBYTE(DXL_HIWORD(dxl_vel_limit[id_count]));
            param_vel_limit[id_count][3] = DXL_HIBYTE(DXL_HIWORD(dxl_vel_limit[id_count]));
    }
    
    // I.7.Add Dynamixel#1 goal velocity value to the Syncwrite storage
for(int id_count = 0; id_count < DxlIds_size; id_count++){
    dxl_addparam_result = groupSyncWriteVelLim.addParam(DxlIDs[id_count], param_vel_limit[id_count]);
    if (dxl_addparam_result != true)
    {
      Serial.print("[ID:"); Serial.print(DxlIDs[id_count]); Serial.println("] groupSyncWriteVelLim addparam failed");
      return false;
    }
    else
    {
      Serial.print("[ID:"); Serial.print(DxlIDs[id_count]); Serial.println("] groupSyncWriteVelLim addparam successful");
    }
}

    // III.c.9.Syncwrite goal velocity
    dxl_comm_result = groupSyncWriteVelLim.txPacket();
    if (dxl_comm_result != COMM_SUCCESS){
      Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
      return false;
    }
    else
    {
      for(int id_count = 0; id_count < DxlIds_size; id_count++){  
        Serial.println("groupSyncWritVelLim txPacket successful");
        Serial.print("[ID:"); Serial.print(DxlIDs[id_count]);  Serial.print("] Velocity Limit CHANGED"); Serial.print(" New Velocity Limit:"); Serial.println(dxl_vel_limit[id_count]);
      }
    }

    // III.c.10.Clear syncwrite parameter storage
    groupSyncWriteVelLim.clearParam();

    return true;
}
else
{
    Serial.println("syncSetVelAccelLimit: Matrices of different size were given FAILED");

    return false;
}

} // END FUNCTION

// =========================================================================================================== //

bool DynamixelProPlusMetamorphicManipulator::syncSetGoalPosition(uint8_t *DxlIDs, int DxlIds_size, dxlGoalPos dxl_goal_pos, int dxl_goal_pos_size, dynamixel::GroupSyncWrite groupSyncWriteGoalPos,  dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler){
 
/*
 * Changes Goal Position Values for all Dynamixels. Torque MUST be ENABLED!
 * Different  Goal Position value can be given to each Dynamixel
 * The number of  Goal Position values specified must equal the dynamixel motors!
 */

// Define 2D arrays for parameter storage
paramGoalPos param_goal_pos[dxl_goal_pos_size];

// Checks if DxlIds_size == 
if ( DxlIds_size == dxl_goal_pos_size)
{
    // I. Set Limit values for Velocity to Dynamixels
    for(int id_count = 0; id_count < DxlIds_size; id_count++){
            // I.1. Allocate Acceleration Limit value into byte array
            param_goal_pos[id_count][0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_pos[id_count]));
            param_goal_pos[id_count][1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_pos[id_count]));
            param_goal_pos[id_count][2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_pos[id_count]));
            param_goal_pos[id_count][3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_pos[id_count]));
    }

for(int id_count = 0; id_count < DxlIds_size; id_count++){
    dxl_addparam_result = groupSyncWriteGoalPos.addParam(DxlIDs[id_count], param_goal_pos[id_count]);
    if (dxl_addparam_result != true)
    {
      Serial.print("[ID:"); Serial.print(DxlIDs[id_count]); Serial.println("] groupSyncWriteGoalPos addparam failed");
      return false;
    }
    else
    {
      Serial.print("[ID:"); Serial.print(DxlIDs[id_count]); Serial.println("] groupSyncWriteGoalPos addparam successful");
    }
}

    // I.4.Syncwrite goal acceleration
    dxl_comm_result = groupSyncWriteGoalPos.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
       Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    }
    else
    {
      for(int id_count = 0; id_count < DxlIds_size; id_count++){  
        Serial.println("groupSyncWriteGoalPos txPacket successful");
        Serial.print("[ID:"); Serial.print(DxlIDs[id_count]);  Serial.print("] Acceleration Limit CHANGED"); Serial.print(" New Acceleration Limit:"); Serial.println(dxl_goal_pos[id_count]);
      }
    }
    
    // I.5.Clear syncwrite parameter storage
    groupSyncWriteGoalPos.clearParam();
    
    return true;
}
else
{
    Serial.println("groupSyncWriteGoalPos: Matrices of different size were given FAILED");

    return false;
}

} // END FUNCTION

// =========================================================================================================== //
