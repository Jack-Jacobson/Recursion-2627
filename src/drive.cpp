
#include "drive.h"


using namespace vex;

double Pgain[5] = {1.3, .03, .5, .2, 0};
double Igain[5] = {0, 0, 0, 0, 0};
double Dgain[5] = {120, 0, 0, 0, 0};

double allPastError[5] = {0, 0, 0, 0, 0};
double lastError[5] = {0, 0, 0, 0, 0};
double lastTimeRan[5] = {0, 0, 0, 0, 0};

vex::timer Timer;

int www = 0;
double liftHight = 0;


void setDrive() {
 int forward = Controller.Axis3.position();
 int sideways = Controller.Axis1.position();

 // deadband to prevent drift
 if (abs(forward) < 5) forward = 0;
 if (abs(sideways) < 5) sideways = 0;

 int leftVelocity = 0.12 * (forward + sideways);
 int rightVelocity = 0.12 * (forward - sideways);

 frontLeftDrive.spin(directionType::fwd,  leftVelocity,  voltageUnits::volt);
  midLeftDrive.spin(directionType::fwd,    leftVelocity,  voltageUnits::volt);
  backLeftDrive.spin(directionType::fwd,   leftVelocity,  voltageUnits::volt);

  frontRightDrive.spin(directionType::fwd, rightVelocity, voltageUnits::volt);
  midRightDrive.spin(directionType::fwd,   rightVelocity, voltageUnits::volt);
  backRightDrive.spin(directionType::fwd,  rightVelocity, voltageUnits::volt);

}

// Hold L2 = up, hold L1 = down, neither = coast so it sinks under gravity
void moveCascade(){
  if (Controller.ButtonL2.pressing()) {
    liftHight = cascadeUpVolts;
  } else if (Controller.ButtonL1.pressing()) {
    liftHight = -cascadeDownVolts;
  } else {
    liftHight = 0;
  }

  if (liftHight == 0) {
    lift1.stop(brakeType::coast);
    lift2.stop(brakeType::coast);
    lift3.stop(brakeType::coast);
    return;
  }

  lift1.spin(directionType::fwd, liftHight, voltageUnits::volt);
  lift2.spin(directionType::fwd, liftHight, voltageUnits::volt);
  lift3.spin(directionType::fwd, liftHight, voltageUnits::volt);
}

bool clawed = false;
void toggleClaw(){
  clawed = !clawed;
  claw.set(clawed);
}

double clamp (double value, double hiClamp, double loClamp, double deadHi, double deadLo) {

    if (value > hiClamp) {
        return hiClamp;
    }
    if (value < loClamp) {
        return loClamp;
    }
    if (value < deadHi && value > deadLo) {
        return 0;
    }

    return value;

}

double PID(double Ti, double error, int Values) {
    // variabels
     double P = 0;
     double I = 0;
     double D = 0;
     double reateOfChange = 0;
     double changeInError = 0;
     double changeInTime = 0;

     //finding P
     P = Pgain[Values] * error;
      
      
     //finding I
      if (error == 0) {
         allPastError[Values] = 0;
      } else {
         allPastError[Values] = error + allPastError[Values];
     }
     I = allPastError[Values] * Igain[Values];              
      
      
     //finding D
     changeInTime = Ti - lastTimeRan[Values];
     if (changeInTime <= 0) {
         return P + I;
     }
     reateOfChange = (error - lastError[Values]) / changeInTime;
     D = reateOfChange * Dgain[Values];
      
     lastError[Values] = error;
     lastTimeRan[Values] = Ti;
      
      
     // refinding D values
     return (P  + I + D);
      
      
}