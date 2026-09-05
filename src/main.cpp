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

struct Pose { double x = 0, y = 0, theta = 0; };
static Pose pose;

const double wheelDiameterMm = 50.8;
const double wheelCircumferenceMm = M_PI * wheelDiameterMm;
double prevOdomDeg = 0.0;
double prevYawDeg = 0.0;

void updateOdom() {
    while(true) {
        double currOdomDeg = odomPod.position(rotationUnits::deg);
        double deltaDeg = currOdomDeg - prevOdomDeg;
        double deltaMm = (deltaDeg / 360.0) * wheelCircumferenceMm;

        double currYawDeg = inertialSensor.heading();
        double deltaYawDeg = currYawDeg - prevYawDeg;
        if(deltaYawDeg > 180) deltaYawDeg -= 360;
        else if(deltaYawDeg < -180) deltaYawDeg += 360;
        double deltaTheta = deltaYawDeg * M_PI / 180.0;

        double midTheta = pose.theta + deltaTheta * 0.5;
        pose.x += deltaMm * cos(midTheta);
        pose.y += deltaMm * sin(midTheta);
        pose.theta += deltaTheta;

        prevOdomDeg = currOdomDeg;
        prevYawDeg = currYawDeg;

        this_thread::sleep_for(10);
    }
}




int main() {

    odomPod.resetPosition();
    this_thread::sleep_for(50);
    prevOdomDeg = odomPod.position(rotationUnits::deg);

    inertialSensor.calibrate();
    while(inertialSensor.isCalibrating()){
        this_thread::sleep_for(10);
        Brain.Screen.print("Calibrating Inertial Sensor...");
    }
    prevYawDeg = inertialSensor.heading();

    vex::thread odomThread(updateOdom);

    while(1) {
        
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        this_thread::sleep_for(20);
    }
}
