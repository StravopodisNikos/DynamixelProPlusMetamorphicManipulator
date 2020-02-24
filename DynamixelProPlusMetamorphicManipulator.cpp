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

// Constructor
DynamixelProPlusMetamorphicManipulator::DynamixelProPlusMetamorphicManipulator(){
}

// Ping Dynamixels
bool DynamixelProPlusMetamorphicManipulator::pingDynamixels(uint8_t *DxlIDs, int DxlIds_size, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler) {
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
