/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       matthandzel                                               */
/*    Created:      11/18/2022, 10:16:23 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;


// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;

// define your global instances of motors and other devices here
vex::digital_out led1(Brain.ThreeWirePort.A);

extern "C" {
    int32_t  vexAdiAddrLedSet( uint32_t index, uint32_t port, uint32_t *pData, uint32_t nOffset, uint32_t nLength, uint32_t options );
}


int main() {

    Brain.Screen.printAt( 10, 50, "Hello V5" );

    
   
    while(1) {
        
        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}
