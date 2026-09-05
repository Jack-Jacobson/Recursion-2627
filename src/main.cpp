/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       jackj                                                     */
/*    Created:      9/2/2026, 6:38:16 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "motorDefs.h"


using namespace vex;

double odomMm = 0.0;
double prevDeg = 0.0;

const double wheelDiameterMm = 50.8;
const double wheelCircumferenceMm = M_PI * wheelDiameterMm;

void updateOdom() {
    while(true){
        double currDeg = odomPod.position(rotationUnits::deg);

        double deltaDeg = currDeg - prevDeg;

        double deltaMm = (deltaDeg / 360.0) * wheelCircumferenceMm;
        odomMm += deltaMm;

        prevDeg = currDeg;
        
        this_thread::sleep_for(10);
    }
    
}

int main() {

    odomPod.resetPosition();
    this_thread::sleep_for(50);
    prevDeg = odomPod.position(rotationUnits::deg);

    vex::thread odomThread(updateOdom);

    while(1) {
        
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Odom: %.2f mm", odomMm);
        this_thread::sleep_for(20);
    }
}
