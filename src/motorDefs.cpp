#include "motorDefs.h"


vex::brain Brain;
vex::controller Controller = vex::controller();
vex::motor frontLeftDrive(vex::PORT9, vex::gearSetting::ratio6_1, false);
vex::motor midLeftDrive(vex::PORT7, vex::gearSetting::ratio6_1, true);
vex::motor backLeftDrive(vex::PORT10, vex::gearSetting::ratio6_1, false);
vex::motor frontRightDrive(vex::PORT1, vex::gearSetting::ratio6_1, false);
vex::motor midRightDrive(vex::PORT3, vex::gearSetting::ratio6_1, true);
vex::motor backRightDrive(vex::PORT2, vex::gearSetting::ratio6_1, false);

vex::rotation odomPod(vex::PORT14, false);
vex::inertial inertialSensor(vex::PORT8);
