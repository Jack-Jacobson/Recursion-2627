#include "motorDefs.h"


vex::brain Brain;
vex::controller Controller = vex::controller();
vex::motor frontLeftDrive(vex::PORT4, vex::gearSetting::ratio6_1, true);
vex::motor midLeftDrive(vex::PORT6, vex::gearSetting::ratio6_1, true);
vex::motor backLeftDrive(vex::PORT5, vex::gearSetting::ratio6_1, false);    
vex::motor frontRightDrive(vex::PORT8, vex::gearSetting::ratio6_1, false);
vex::motor midRightDrive(vex::PORT10, vex::gearSetting::ratio6_1, false);
vex::motor backRightDrive(vex::PORT9, vex::gearSetting::ratio6_1, true);

vex::rotation odomPod(vex::PORT20, false);
vex::inertial inertialSensor(vex::PORT19);

vex::digital_out claw = vex::digital_out(Brain.ThreeWirePort.A);
