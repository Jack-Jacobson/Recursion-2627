
#include "drive.h"
#include "lift.h"


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
 if (std::abs(static_cast<double>(forward)) < 5.0) forward = 0;
 if (std::abs(static_cast<double>(sideways)) < 5.0) sideways = 0;

 int leftVelocity = 0.12 * (forward + sideways);
 int rightVelocity = 0.12 * (forward - sideways);

 // Sticks released: coast instead of holding/braking (autonomous may leave hold set).
 if (forward == 0 && sideways == 0) {
   frontLeftDrive.stop(brakeType::coast);
   midLeftDrive.stop(brakeType::coast);
   backLeftDrive.stop(brakeType::coast);
   frontRightDrive.stop(brakeType::coast);
   midRightDrive.stop(brakeType::coast);
   backRightDrive.stop(brakeType::coast);
   return;
 }

 frontLeftDrive.spin(directionType::fwd,  leftVelocity,  voltageUnits::volt);
  midLeftDrive.spin(directionType::fwd,    leftVelocity,  voltageUnits::volt);
  backLeftDrive.spin(directionType::fwd,   leftVelocity,  voltageUnits::volt);

  frontRightDrive.spin(directionType::fwd, rightVelocity, voltageUnits::volt);
  midRightDrive.spin(directionType::fwd,   rightVelocity, voltageUnits::volt);
  backRightDrive.spin(directionType::fwd,  rightVelocity, voltageUnits::volt);

}
// Hold L2: raise while held and clear coast lockout.
// L1: force coast mode immediately; while held, drive down.
void moveCascade(){
  static bool coastMode = false;
  static bool brakeLockedOut = false;

  bool upHeld = Controller.ButtonL2.pressing();
  bool downHeld = Controller.ButtonL1.pressing();

  // Up takes priority if both are pressed.
  if (upHeld) {
    setLiftControl(false);
    coastMode = false;
    brakeLockedOut = false;
    lift1.spin(directionType::fwd, -cascadeUpVolts, voltageUnits::volt);
    lift2.spin(directionType::fwd, -cascadeUpVolts, voltageUnits::volt);
    lift3.spin(directionType::fwd, -cascadeUpVolts, voltageUnits::volt);
    return;
  }

  if (downHeld) {
    brakeLockedOut = true;
    setLiftControl(false);
    coastMode = true;
    lift1.setStopping(brakeType::coast);
    lift2.setStopping(brakeType::coast);
    lift3.setStopping(brakeType::coast);
    lift1.spin(directionType::fwd, cascadeDownVolts, voltageUnits::volt);
    lift2.spin(directionType::fwd, cascadeDownVolts, voltageUnits::volt);
    lift3.spin(directionType::fwd, cascadeDownVolts, voltageUnits::volt);
    return;
  }

  // A macro owns the lift via PID; don't fight it with stop().
  if (isLiftControlEnabled()) return;

  if (coastMode || brakeLockedOut) {
    lift1.stop(brakeType::coast);
    lift2.stop(brakeType::coast);
    lift3.stop(brakeType::coast);
  } else {
    lift1.stop(brakeType::brake);
    lift2.stop(brakeType::brake);
    lift3.stop(brakeType::brake);
  }
}

bool clawed = true;
void setClawState(bool closed){
  clawed = closed;
  claw.set(closed);
}

bool isClawClosed(){
  return clawed;
}

void toggleClaw(){
  setClawState(!clawed);
}

bool rightToggled = false;
void rightToggle(){
  rightToggled = !rightToggled;
  rightToggler.set(rightToggled);
}

bool leftToggled = false;
void leftToggle(){
  leftToggled = !leftToggled;
  leftToggler.set(leftToggled);
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