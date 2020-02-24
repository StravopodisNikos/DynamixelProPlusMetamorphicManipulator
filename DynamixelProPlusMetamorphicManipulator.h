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

class DynamixelProPlusMetamorphicManipulator
{ 
 public:

    DynamixelProPlusMetamorphicManipulator();


    bool pingDynamixels(uint8_t *DxlIDs, int DxlIds_size, dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler);

    private:

};

 #endif