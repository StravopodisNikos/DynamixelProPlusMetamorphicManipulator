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

// Include Motor Configuration files from folder ~/Arduino/libraries/test_metamorphic_manipulator_configuration
#include <definitions.h>                            
#include <motorIDs.h>                               
#include <contolTableItems_LimitValues.h>
#include <StepperMotorSettings.h>

using namespace std;

// Creating a ney type for storaging 2D arrays of parameter values for control table items
typedef uint8_t paramAccelLimit[LEN_PRO_ACCEL_LIMIT];                                           
typedef uint8_t paramVelLimit[LEN_PRO_VEL_LIMIT];
typedef uint8_t paramGoalPos[LEN_PRO_GOAL_POSITION];
typedef uint8_t type_param_indirect_data_for_GP_A_V_LED[LEN_PRO_INDIRECTDATA_FOR_WRITE_GP_A_V_LED];

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
        uint8_t param_torque_enable = TORQUE_DISABLE;
        Serial.println("syncSet_GP_A_V_LED: Setting Dynamixels TORQUE -> DISABLED to write Indirect data to EEPROM Memory");
        return_function_state = DynamixelProPlusMetamorphicManipulator::syncSetTorque(DxlIDs, sizeof(DxlIDs), param_torque_enable, groupSyncWrite_TORQUE_ENABLE, packetHandler);
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
                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE_GP_A_V_LED + 0, ADDR_PRO_GOAL_POSITION + 0, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }
                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE_GP_A_V_LED + 2, ADDR_PRO_GOAL_POSITION + 1, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }

                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE_GP_A_V_LED + 4, ADDR_PRO_GOAL_POSITION + 2, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }

                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE_GP_A_V_LED + 6, ADDR_PRO_GOAL_POSITION + 3, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }

                // III.2.2. Allocate Profile Acceleration value into byte array
                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE_GP_A_V_LED + 8, ADDR_PRO_PROF_ACCEL + 0, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }
                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE_GP_A_V_LED + 10, ADDR_PRO_PROF_ACCEL + 1, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }

                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE_GP_A_V_LED + 12, ADDR_PRO_PROF_ACCEL + 2, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }

                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE_GP_A_V_LED + 14, ADDR_PRO_PROF_ACCEL + 3, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }

                // III.2.3. Allocate Profile Velocity value into byte array
                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE_GP_A_V_LED + 16, ADDR_PRO_PROF_VEL + 0, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }
                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE_GP_A_V_LED + 18, ADDR_PRO_PROF_VEL + 1, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }

                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE_GP_A_V_LED + 20, ADDR_PRO_PROF_VEL + 2, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }

                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE_GP_A_V_LED + 22, ADDR_PRO_PROF_VEL + 3, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }

                // III.2.3. Allocate LEDs value into byte array
                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE_GP_A_V_LED + 24, ADDR_PRO_LED_BLUE, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }
                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE_GP_A_V_LED + 26, ADDR_PRO_LED_GREEN, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    packetHandler->getTxRxResult(dxl_comm_result);
                }
                else if (dxl_error != 0)
                {
                    packetHandler->getRxPacketError(dxl_error);
                }
                dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_WRITE_GP_A_V_LED + 28, ADDR_PRO_LED_RED, &dxl_error);
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
        param_torque_enable = TORQUE_ENABLE;
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
} // END FUNCTION

// =========================================================================================================== //

bool DynamixelProPlusMetamorphicManipulator::syncGet_PP_MV( uint8_t *DxlIDs, int DxlIds_size, uint8_t *dxl_moving, int dxl_moving_size, uint32_t *dxl_present_position, int dxl_present_position_size , dynamixel::GroupSyncRead groupSyncRead_PP_MV, dynamixel::GroupSyncWrite groupSyncWrite_TORQUE_ENABLE, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler){

/*
 *  Uses Indirect Addressing to read Present Position, Moving.
 *  Accepts pointer to arrays of Dynamixels' Present Position Values/Moving -> Writes corresponding Values
 *  Used to check if all Dynamixels have stopped moving and to get current absolute position values of motors
 */

if ( (DxlIds_size == dxl_moving_size) && (DxlIds_size == dxl_present_position_size) )
{
    //  Disable Dynamixel Torque because Indirect address would not be accessible when the torque is already enabled
        uint8_t param_torque_enable = 0;
        Serial.println("syncSet_GP_A_V_LED: Setting Dynamixels TORQUE -> DISABLED to write Indirect data to EEPROM Memory");
        return_function_state = DynamixelProPlusMetamorphicManipulator::syncSetTorque(DxlIDs, sizeof(DxlIDs), param_torque_enable, groupSyncWrite_TORQUE_ENABLE, packetHandler);
        if (return_function_state == true)
        {
          Serial.println("SUCCESS");
        }
        else
        {
          Serial.println("FAILED");
          return false;
        }

    //  WRITE TO EEPROM
    for(int id_count = 0; id_count < DxlIds_size; id_count++){
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_MV + 0, ADDR_PRO_PRESENT_POSITION + 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
        packetHandler->getRxPacketError(dxl_error);
        }

        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_MV + 2, ADDR_PRO_PRESENT_POSITION + 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
        packetHandler->getRxPacketError(dxl_error);
        }

        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_MV + 4, ADDR_PRO_PRESENT_POSITION + 2, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
        packetHandler->getRxPacketError(dxl_error);
        }

        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_MV + 6, ADDR_PRO_PRESENT_POSITION + 3, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
        packetHandler->getRxPacketError(dxl_error);
        }

        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_MV + 8, ADDR_PRO_MOVING, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
        packetHandler->getRxPacketError(dxl_error);
        }
    }

    // Enable Dynamixel Torque since writing to EEPROM Memory finished
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

    // I.   Add parameter storage for the Present Positing & Moving items
    for(int id_count = 0; id_count < DxlIds_size; id_count++){
        dxl_addparam_result = groupSyncRead_PP_MV.addParam(DxlIDs[id_count]);
        if (dxl_addparam_result != true)
        {
            Serial.print("[ ID: "); Serial.print(DxlIDs[id_count]); Serial.println(" ] groupSyncRead_PP_MV.addParam FAILED");
            return false;
        }
        else
        {
            Serial.print("[ ID: "); Serial.print(DxlIDs[id_count]); Serial.println(" ] groupSyncRead_PP_MV.addParam SUCCESS");
        }
    }

    // II.  Start communication 
    dxl_comm_result = groupSyncRead_PP_MV.txRxPacket();
    Serial.print("groupSyncRead_PP_MV.txRxPacket(): dxl_comm_result = "); Serial.println(dxl_comm_result);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        packetHandler->getTxRxResult(dxl_comm_result);
        for(int id_count = 0; id_count < DxlIds_size; id_count++){
            Serial.print("[ ID: "); Serial.print(DxlIDs[id_count]); Serial.println(" ] groupSyncRead_PP_MV.txRxPacket() FAILED");
            return false;
        }
    }
    else
    { 
        for(int id_count = 0; id_count < DxlIds_size; id_count++){
            Serial.print("[ID: "); Serial.print(DxlIDs[id_count]); Serial.println(" ] groupSyncRead_PP_MV.txRxPacket() SUCCESS");
        }

    }

    // III. Check if data is available
    for(int id_count = 0; id_count < DxlIds_size; id_count++)
    {
        dxl_getdata_result = groupSyncRead_PP_MV.isAvailable(DxlIDs[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ_PP_MV, LEN_PRO_PRESENT_POSITION);
        if (dxl_getdata_result != true)
        {
            Serial.print("[ID: "); Serial.print(DxlIDs[id_count]); Serial.println("] groupSyncRead_PP_MV.isAvailable PRESENT_POSITION FAILED");
            return false;
        }
        dxl_getdata_result = groupSyncRead_PP_MV.isAvailable(DxlIDs[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ_PP_MV + LEN_PRO_PRESENT_POSITION, LEN_PRO_MOVING);
        if (dxl_getdata_result != true)
        {
            Serial.print("[ID: "); Serial.print(DxlIDs[id_count]); Serial.println("] groupSyncRead_PP_MV.isAvailable MOVING FAILED");
            return false;
        }
    }

    // IV.  Get data
    for(int id_count = 0; id_count < DxlIds_size; id_count++)
    {
        dxl_present_position[id_count] = groupSyncRead_PP_MV.getData(DxlIDs[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ_PP_MV, LEN_PRO_PRESENT_POSITION);
        
        dxl_moving[id_count] = groupSyncRead_PP_MV.getData(DxlIDs[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ_PP_MV + LEN_PRO_PRESENT_POSITION, LEN_PRO_MOVING);
    }

    // V.   Print success
    Serial.println("syncGet_PP_MV: SUCCESS");
    return true;
}
else
{
    // Only prints failed
    Serial.println("syncGet_PP_MV: Arrays of different size were given FAILED");
    return false;
}

// =========================================================================================================== //

} // END FUNCTION

bool DynamixelProPlusMetamorphicManipulator::syncGet_PP_PV_PA_VL_AL( uint8_t *DxlIDs, int DxlIds_size, uint32_t *dxl_present_position, int dxl_present_position_size , int32_t dxl_prof_vel[], int dxl_prof_vel_size, int32_t dxl_prof_accel[], int dxl_prof_accel_size , int32_t dxl_vel_limit[], int dxl_vel_limit_size,  int32_t dxl_accel_limit[], int dxl_accel_limit_size, dynamixel::GroupSyncRead groupSyncRead_PP_PV_PA_VL_AL, dynamixel::GroupSyncWrite groupSyncWrite_TORQUE_ENABLE, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler) {
/*
    *  Reads 1. Present Absolute Position
    *        2. Profile Acceleration
    *        3. Profile Velocity
    *        4. Velocity Limit
    *        5. Acceleration Limit
    *  Always executed at setup
    *  "Twin" function of CustomStepperMetamorphicManipulator/readEEPROMsettings
    */
if ( (DxlIds_size == dxl_present_position_size) && (DxlIds_size == dxl_prof_vel_size) && (DxlIds_size == dxl_prof_accel_size) && (DxlIds_size == dxl_vel_limit_size) && (DxlIds_size == dxl_accel_limit_size) )
{
    // 1. Disable Dynamixel Torque because Indirect address would not be accessible when the torque is already enabled
    uint8_t param_torque_enable = TORQUE_DISABLE;
    Serial.println("[syncGet_PP_PV_PA_VL_AL ]: Setting Dynamixels TORQUE -> DISABLED to write Indirect Data to EEPROM Memory");
    return_function_state = DynamixelProPlusMetamorphicManipulator::syncSetTorque(DxlIDs, sizeof(DxlIDs), param_torque_enable, groupSyncWrite_TORQUE_ENABLE, packetHandler);
    if (return_function_state == true)
    {
        Serial.println("SUCCESS");
    }
    else
    {
        Serial.println("FAILED");
        return false;
    }
    
    // 2. Write to EEPROM the INDIRECTDATA parameter storage
    for(int id_count = 0; id_count < DxlIds_size; id_count++){
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_PV_PA_VL_AL + 0, ADDR_PRO_PRESENT_POSITION + 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
        packetHandler->getRxPacketError(dxl_error);
        }

        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_PV_PA_VL_AL + 2, ADDR_PRO_PRESENT_POSITION + 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
        packetHandler->getRxPacketError(dxl_error);
        }

        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_PV_PA_VL_AL + 4, ADDR_PRO_PRESENT_POSITION + 2, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
        packetHandler->getRxPacketError(dxl_error);
        }

        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_PV_PA_VL_AL + 6, ADDR_PRO_PRESENT_POSITION + 3, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
        packetHandler->getRxPacketError(dxl_error);
        }

        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_PV_PA_VL_AL + 8, ADDR_PRO_PROF_VEL + 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
        packetHandler->getRxPacketError(dxl_error);
        }

        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_PV_PA_VL_AL + 10, ADDR_PRO_PROF_VEL + 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
        packetHandler->getRxPacketError(dxl_error);
        }
        
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_PV_PA_VL_AL + 12, ADDR_PRO_PROF_VEL + 2, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
        packetHandler->getRxPacketError(dxl_error);
        }
        
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_PV_PA_VL_AL + 14, ADDR_PRO_PROF_VEL + 3, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
        packetHandler->getRxPacketError(dxl_error);
        }
        
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_PV_PA_VL_AL + 16, ADDR_PRO_PROF_ACCEL + 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
        packetHandler->getRxPacketError(dxl_error);
        }
        
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_PV_PA_VL_AL + 18, ADDR_PRO_PROF_ACCEL + 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
        packetHandler->getRxPacketError(dxl_error);
        }
        
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_PV_PA_VL_AL + 20, ADDR_PRO_PROF_ACCEL + 2, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
        packetHandler->getRxPacketError(dxl_error);
        }
        
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_PV_PA_VL_AL + 22, ADDR_PRO_PROF_ACCEL + 3, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
        packetHandler->getRxPacketError(dxl_error);
        }
        
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_PV_PA_VL_AL + 24, ADDR_PRO_VEL_LIMIT + 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
        packetHandler->getRxPacketError(dxl_error);
        }
        
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_PV_PA_VL_AL + 26, ADDR_PRO_VEL_LIMIT + 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
        packetHandler->getRxPacketError(dxl_error);
        }
        
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_PV_PA_VL_AL + 28, ADDR_PRO_VEL_LIMIT + 2, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
        packetHandler->getRxPacketError(dxl_error);
        }
        
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_PV_PA_VL_AL + 30, ADDR_PRO_VEL_LIMIT + 3, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
        packetHandler->getRxPacketError(dxl_error);
        }
        
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_PV_PA_VL_AL + 32, ADDR_PRO_ACCEL_LIMIT + 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
        packetHandler->getRxPacketError(dxl_error);
        }

        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_PV_PA_VL_AL + 34, ADDR_PRO_ACCEL_LIMIT + 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
        packetHandler->getRxPacketError(dxl_error);
        }   

        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_PV_PA_VL_AL + 36, ADDR_PRO_ACCEL_LIMIT + 2, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
        packetHandler->getRxPacketError(dxl_error);
        }

        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DxlIDs[id_count], ADDR_PRO_INDIRECTADDRESS_FOR_READ_PP_PV_PA_VL_AL + 38, ADDR_PRO_ACCEL_LIMIT + 3, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
        packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
        packetHandler->getRxPacketError(dxl_error);
        } 
    }

    // 3. Enable Dynamixel Torque since writing to EEPROM Memory finished
    param_torque_enable = TORQUE_ENABLE;
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

    // 4.   Add parameter storage for the Present Positing & Moving items
    for(int id_count = 0; id_count < DxlIds_size; id_count++){
        dxl_addparam_result = groupSyncRead_PP_PV_PA_VL_AL.addParam(DxlIDs[id_count]);
        if (dxl_addparam_result != true)
        {
            Serial.print("[ ID: "); Serial.print(DxlIDs[id_count]); Serial.println(" ] groupSyncRead_PP_PV_PA_VL_AL.addParam FAILED");
            return false;
        }
        else
        {
            Serial.print("[ ID: "); Serial.print(DxlIDs[id_count]); Serial.println(" ] groupSyncRead_PP_PV_PA_VL_AL.addParam SUCCESS");
        }
    }

    // 5.  Start communication 
    dxl_comm_result = groupSyncRead_PP_PV_PA_VL_AL.txRxPacket();
    Serial.print("groupSyncRead_PP_MV.txRxPacket(): dxl_comm_result = "); Serial.println(dxl_comm_result);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        packetHandler->getTxRxResult(dxl_comm_result);
        for(int id_count = 0; id_count < DxlIds_size; id_count++){
            Serial.print("[ ID: "); Serial.print(DxlIDs[id_count]); Serial.println(" ] groupSyncRead_PP_PV_PA_VL_AL.txRxPacket() FAILED");
            return false;
        }
    }
    else
    { 
        for(int id_count = 0; id_count < DxlIds_size; id_count++){
            Serial.print("[ID: "); Serial.print(DxlIDs[id_count]); Serial.println(" ] groupSyncRead_PP_PV_PA_VL_AL.txRxPacket() SUCCESS");
        }

    }

    // 6. Check if data is available
    for(int id_count = 0; id_count < DxlIds_size; id_count++)
    {
        dxl_getdata_result = groupSyncRead_PP_PV_PA_VL_AL.isAvailable(DxlIDs[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ_PP_PV_PA_VL_AL, LEN_PRO_PRESENT_POSITION);
        if (dxl_getdata_result != true)
        {
            Serial.print("[ID: "); Serial.print(DxlIDs[id_count]); Serial.println("] groupSyncRead_PP_PV_PA_VL_AL.isAvailable PRESENT_POSITION FAILED");
            return false;
        }
        dxl_getdata_result = groupSyncRead_PP_PV_PA_VL_AL.isAvailable(DxlIDs[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ_PP_PV_PA_VL_AL + LEN_PRO_PRESENT_POSITION, LEN_PRO_PROF_VEL);
        if (dxl_getdata_result != true)
        {
            Serial.print("[ID: "); Serial.print(DxlIDs[id_count]); Serial.println("] groupSyncRead_PP_PV_PA_VL_AL.isAvailable PROFILE_VELOCITY FAILED");
            return false;
        }
        dxl_getdata_result = groupSyncRead_PP_PV_PA_VL_AL.isAvailable(DxlIDs[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ_PP_PV_PA_VL_AL + LEN_PRO_PRESENT_POSITION + LEN_PRO_PROF_VEL, LEN_PRO_PROF_ACCEL);
        if (dxl_getdata_result != true)
        {
            Serial.print("[ID: "); Serial.print(DxlIDs[id_count]); Serial.println("] groupSyncRead_PP_PV_PA_VL_AL.isAvailable PROFILE_ACCELERATION FAILED");
            return false;
        }
        dxl_getdata_result = groupSyncRead_PP_PV_PA_VL_AL.isAvailable(DxlIDs[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ_PP_PV_PA_VL_AL + LEN_PRO_PRESENT_POSITION + LEN_PRO_PROF_VEL + LEN_PRO_PROF_ACCEL, LEN_PRO_VEL_LIMIT);
        if (dxl_getdata_result != true)
        {
            Serial.print("[ID: "); Serial.print(DxlIDs[id_count]); Serial.println("] groupSyncRead_PP_PV_PA_VL_AL.isAvailable VELOCITY_LIMIT FAILED");
            return false;
        }
        dxl_getdata_result = groupSyncRead_PP_PV_PA_VL_AL.isAvailable(DxlIDs[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ_PP_PV_PA_VL_AL + LEN_PRO_PRESENT_POSITION + LEN_PRO_PROF_VEL + LEN_PRO_PROF_ACCEL + LEN_PRO_VEL_LIMIT, LEN_PRO_ACCEL_LIMIT);
        if (dxl_getdata_result != true)
        {
            Serial.print("[ID: "); Serial.print(DxlIDs[id_count]); Serial.println("] groupSyncRead_PP_PV_PA_VL_AL.isAvailable ACCELERATION_LIMIT FAILED");
            return false;
        }
    }

    // 7.  Get data
    for(int id_count = 0; id_count < DxlIds_size; id_count++)
    {
        dxl_present_position[id_count] = groupSyncRead_PP_PV_PA_VL_AL.getData(DxlIDs[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ_PP_PV_PA_VL_AL, LEN_PRO_PRESENT_POSITION);
        
        dxl_prof_vel[id_count] = groupSyncRead_PP_PV_PA_VL_AL.getData(DxlIDs[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ_PP_PV_PA_VL_AL + LEN_PRO_PRESENT_POSITION, LEN_PRO_PROF_VEL);
    
        dxl_prof_accel[id_count] = groupSyncRead_PP_PV_PA_VL_AL.getData(DxlIDs[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ_PP_PV_PA_VL_AL + LEN_PRO_PRESENT_POSITION + LEN_PRO_PROF_VEL, LEN_PRO_PROF_ACCEL);

        dxl_vel_limit[id_count] = groupSyncRead_PP_PV_PA_VL_AL.getData(DxlIDs[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ_PP_PV_PA_VL_AL + LEN_PRO_PRESENT_POSITION + LEN_PRO_PROF_VEL + LEN_PRO_PROF_ACCEL, LEN_PRO_VEL_LIMIT);

        dxl_accel_limit[id_count] = groupSyncRead_PP_PV_PA_VL_AL.getData(DxlIDs[id_count], ADDR_PRO_INDIRECTDATA_FOR_READ_PP_PV_PA_VL_AL + LEN_PRO_PRESENT_POSITION + LEN_PRO_PROF_VEL + LEN_PRO_PROF_ACCEL + LEN_PRO_VEL_LIMIT, LEN_PRO_ACCEL_LIMIT);

    }

    // 8.   Print success
    Serial.println("groupSyncRead_PP_PV_PA_VL_AL: SUCCESS");
    return true;

}
else
{
    // Only prints failed
    Serial.println("[ syncGet_PP_PV_PA_VL_AL ]: Arrays of different size were given FAILED");
    return false;
}

}

int32_t DynamixelProPlusMetamorphicManipulator::convertRadian2DxlPulses(double position_in_radians)
{
    int32_t position_in_dxl_pulses;
    //double position_in_radians;

    if (position_in_radians == 0)
    {
        position_in_dxl_pulses = 0;
    }
    else 
    {
        position_in_dxl_pulses = (position_in_radians * DXL_RESOLUTION)/PI;
    }

return position_in_dxl_pulses;
}
  
double DynamixelProPlusMetamorphicManipulator::convertDxlPulses2Radian(int32_t position_in_dxl_pulses)
{
    double position_in_radians;
    
    if (position_in_dxl_pulses == 0)
    {
        position_in_radians = (double) position_in_dxl_pulses;
    }
    else
    {
        position_in_radians = (double) (position_in_dxl_pulses * PI) / DXL_RESOLUTION ;
    }
    
    return position_in_radians;
}