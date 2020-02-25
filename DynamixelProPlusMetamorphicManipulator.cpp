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
#include "definitions.h"                            // Includes definitions of control table variables addresses/data lengths/ communication/ protocols
#include "motorIDs.h"                               // Includes motor IDs as set using Dynamixel Wizard

using namespace std;

// Creating a ney type for storaging 2D arrays of parameter values for control table items
typedef uint8_t paramAccelLimit[LEN_PRO_ACCEL_LIMIT];                                           
typedef uint8_t paramVelLimit[LEN_PRO_VEL_LIMIT];
typedef uint8_t paramGoalPos[LEN_PRO_GOAL_POSITION];
typedef uint8_t type_param_indirect_data_for_GP_A_V_LED[LEN_PRO_INDIRECTDATA_FOR_WRITE];

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

bool DynamixelProPlusMetamorphicManipulator::syncSetTorque(uint8_t *DxlIDs, int DxlIds_size, uint8_t data, dynamixel::GroupSyncWrite groupSyncWrite_TORQUE_ENABLE, dynamixel::PacketHandler *packetHandler)
{     
/*
 *  Sets the same toque status to all connected Dynamixels
 * torque status value is given through data variable and range value is: 0~1 
 */
    for(int id_count = 0; id_count < DxlIds_size; id_count++){
        dxl_addparam_result = groupSyncWrite_TORQUE_ENABLE.addParam(DxlIDs[id_count], &data);
        if (dxl_addparam_result != true)
        {
          Serial.print("[ID:"); Serial.print(DxlIDs[id_count]); Serial.println("groupSyncWrite_TORQUE_ENABLE.addParam FAILED");
          return false;
        }
    }

    dxl_comm_result = groupSyncWrite_TORQUE_ENABLE.txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(packetHandler->getTxRxResult(dxl_comm_result)); Serial.println("groupSyncWrite_TORQUE_ENABLE.txPacket() FAILED");
      return false;
    }
    else
    {
      Serial.print("Torque STATUS CHANGED"); Serial.print(" New status:"); Serial.println(data);
    }
    
    groupSyncWrite_TORQUE_ENABLE.clearParam();
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

bool DynamixelProPlusMetamorphicManipulator::syncSet_GP_A_V_LED( uint8_t *DxlIDs, int DxlIds_size, typeDxlTrapzProfParams DxlTrapzProfParams[], int DxlTrapzProfParams_size, dynamixel::GroupSyncWrite groupSyncWrite_GP_A_V_LED, dynamixel::GroupSyncWrite groupSyncWrite_TORQUE_ENABLE, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler){

/*
 *  Uses Indirect Addressing to write Goal Position, Acceleration, Velocity and LED values
 */

// I. Disable Dynamixel Torque because Indirect address would not be accessible when the torque is already enabled
        uint8_t param_torque_enable = 0;
        Serial.println("syncSet_GP_A_V_LED: Setting Dynamixels TORQUE -> DISABLED to write Indirect data to EEPROM Memory");
        return_function_state = syncSetTorque(DxlIDs, sizeof(DxlIDs), param_torque_enable, groupSyncWrite_TORQUE_ENABLE, packetHandler);
        if (return_function_state == true)
        {
          Serial.println("SUCCESS");
        }
        else
        {
          Serial.println("FAILED");
        }
// II. Allocatec Values to Write into byte arrays

        for(int id_count = 0; id_count < DxlIds_size; id_count++){
                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 0, ADDR_PRO_GOAL_POSITION + 0, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }
                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 2, ADDR_PRO_GOAL_POSITION + 1, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }

                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 4, ADDR_PRO_GOAL_POSITION + 2, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }

                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 6, ADDR_PRO_GOAL_POSITION + 3, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }

                // III.2.2. Allocate Profile Acceleration value into byte array
                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 8, ADDR_PRO_PROF_ACCEL + 0, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }
                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 10, ADDR_PRO_PROF_ACCEL + 1, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }

                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 12, ADDR_PRO_PROF_ACCEL + 2, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }

                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 14, ADDR_PRO_PROF_ACCEL + 3, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }

                // III.2.3. Allocate Profile Velocity value into byte array
                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 16, ADDR_PRO_PROF_VEL + 0, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }
                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 18, ADDR_PRO_PROF_VEL + 1, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }

                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 20, ADDR_PRO_PROF_VEL + 2, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }

                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 22, ADDR_PRO_PROF_VEL + 3, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }

                // III.2.3. Allocate LEDs value into byte array
                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 24, ADDR_PRO_LED_BLUE, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }
                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 26, ADDR_PRO_LED_GREEN, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }
                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE + 28, ADDR_PRO_LED_RED, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }
        } // FINISHED WRITING IN EEPROM MEMORY

// III. Enable Dynamixel Torque since writing to EEPROM Memory finished
        param_torque_enable = 1;
        Serial.println("Setting Dynamixels TORQUE: ENABLED since finished accessing EEPROM.");
        return_function_state = syncSetTorque(DxlIDs, DxlIds_size, param_torque_enable, groupSyncWrite_TORQUE_ENABLE, packetHandler);
        if (return_function_state == true)
        {
          Serial.println("SUCCESS");
        }
        else
        {
          Serial.println("FAILED");
        }

type_param_indirect_data_for_GP_A_V_LED param_indirect_data_for_GP_A_V_LED[DxlTrapzProfParams_size];

        for(int id_count = 0; id_count < DxlIds_size; id_count++){
                // IV.  Allocate parameters for write to each Dynamixel
                param_indirect_data_for_GP_A_V_LED[id_count][0] = DXL_LOBYTE(DXL_LOWORD(DxlTrapzProfParams[id_count][1]));      // Goal Position
                param_indirect_data_for_GP_A_V_LED[id_count][1] = DXL_HIBYTE(DXL_LOWORD(DxlTrapzProfParams[id_count][1]));
                param_indirect_data_for_GP_A_V_LED[id_count][2] = DXL_LOBYTE(DXL_HIWORD(DxlTrapzProfParams[id_count][1]));
                param_indirect_data_for_GP_A_V_LED[id_count][3] = DXL_HIBYTE(DXL_HIWORD(DxlTrapzProfParams[id_count][1]));
                
                param_indirect_data_for_GP_A_V_LED[id_count][4] = DXL_LOBYTE(DXL_LOWORD(DxlTrapzProfParams[id_count][2]));      // Goal Velocity
                param_indirect_data_for_GP_A_V_LED[id_count][5] = DXL_HIBYTE(DXL_LOWORD(DxlTrapzProfParams[id_count][2]));
                param_indirect_data_for_GP_A_V_LED[id_count][6] = DXL_LOBYTE(DXL_HIWORD(DxlTrapzProfParams[id_count][2]));
                param_indirect_data_for_GP_A_V_LED[id_count][7] = DXL_HIBYTE(DXL_HIWORD(DxlTrapzProfParams[id_count][2]));
                
                param_indirect_data_for_GP_A_V_LED[id_count][8] = DXL_LOBYTE(DXL_LOWORD(DxlTrapzProfParams[id_count][3]));      // Goal Acceleration
                param_indirect_data_for_GP_A_V_LED[id_count][9] = DXL_HIBYTE(DXL_LOWORD(DxlTrapzProfParams[id_count][3]));
                param_indirect_data_for_GP_A_V_LED[id_count][10] = DXL_LOBYTE(DXL_HIWORD(DxlTrapzProfParams[id_count][3]));
                param_indirect_data_for_GP_A_V_LED[id_count][11] = DXL_HIBYTE(DXL_HIWORD(DxlTrapzProfParams[id_count][3]));
                
                param_indirect_data_for_GP_A_V_LED[id_count][12] = dxl_ledBLUE_value[led_change];                                // BLUE
                param_indirect_data_for_GP_A_V_LED[id_count][13] = dxl_ledGREEN_value[led_change];                               // GREEN
                param_indirect_data_for_GP_A_V_LED[id_count][14] = dxl_ledRED_value[led_change];                                 // RED

                // V.   Add WRITE values to the Syncwrite parameter storage for each Dynamixel
                dxl_addparam_result = groupSyncWrite_GP_A_V_LED.addParam(DxlIDs[id_count], param_indirect_data_for_GP_A_V_LED[id_count]);
                if (dxl_addparam_result != true)
                {
                    Serial.print("[ID:"); Serial.print(DxlIDs[id_count]); Serial.println("] groupSyncWrite_GP_A_V_LED.addParam addparam FAILED");
                    return false;
                }
                else
                { 
                    Serial.print("[ID:"); Serial.print(DxlIDs[id_count]); Serial.println("] groupSyncWrite_GP_A_V_LED.addParam addparam SUCCESS");
                }
        }

        // VI. Syncwrite Packet is sent to Dynamixels
        dxl_comm_result = groupSyncWrite_GP_A_V_LED.txPacket();
        if (dxl_comm_result != COMM_SUCCESS){
            
            Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
            Serial.println("groupSyncWrite_GP_A_V_LED.txPacket: FAILED");
            groupSyncWrite_GP_A_V_LED.clearParam();                                 // Clears syncwrite parameter storage
            return false;
        }else{
            groupSyncWrite_GP_A_V_LED.clearParam();
            Serial.println("groupSyncWrite_GP_A_V_LED.txPacket: SUCCESS");
            return true;
        }
}