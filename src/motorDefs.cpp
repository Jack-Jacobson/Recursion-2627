#include "motorDefs.h"


vex::brain Brain;
vex::controller Controller = vex::controller();
vex::motor frontLeftDrive(vex::PORT1, vex::gearSetting::ratio6_1, false);
vex::motor midLeftDrive(vex::PORT1, vex::gearSetting::ratio6_1, false);
vex::motor backLeftDrive(vex::PORT1, vex::gearSetting::ratio6_1, false);
vex::motor frontRightDrive(vex::PORT1, vex::gearSetting::ratio6_1, false);
vex::motor midRightDrive(vex::PORT1, vex::gearSetting::ratio6_1, false);
vex::motor backRightDrive(vex::PORT1, vex::gearSetting::ratio6_1, false);