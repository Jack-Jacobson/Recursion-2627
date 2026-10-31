/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       jackj                                                     */
/*    Created:      10/22/2026, 3:40:26 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "drive.h"
#include "lift.h"
#include "brainUI.h"
#include "pursuit.h"

using namespace vex;

competition Competition;


void pre_auton(void) {
  recordLiftHome();
  vex::thread UIThread(brainUI);
  claw.set(true);
  initOdom();
  vex::thread odomThread(updateOdom);
  startLift();
}


void autonomous(void) {
  
  startRunTimer();
  roller.set(true);
  wait(0.3, sec);
  setLiftTarget(110);
  followPath (1, PathOpts().noStartTurn());
  liftTo(40, 600);
  claw.set(false);
  driveDistance(-400, 800, 300);
  setLiftTarget(-20);
  followPath(2, PathOpts());
  claw.set(true);
  wait(0.2, sec);
  setLiftTarget(219);
  followPath(3, PathOpts().endTol(110));
  liftTo(190, 400);
  claw.set(false);
  setLiftTarget(0);
  driveDistance(-300);
  followPath(4, PathOpts());
  claw.set(true);
  setLiftTarget(380);
  wait(0.3, sec);
  driveDistance(-500);
  followPath(5);
  liftTo(360);
  claw.set(false);
  driveDistance(-300);
  setLiftTarget(0); 
  followPath(6);
  claw.set(true);
  wait(0.4, sec);
  setLiftTarget(20);
  driveDistance(-275);  
  setLiftTarget(560);
  followPath(7);
  liftTo(520);
  claw.set(false);
  turnToHeading(100);
  driveDistance(-370);
  setLiftTarget(0);
  followPath(8);
  claw.set(true);
  wait(0.4, sec);
  setLiftTarget(20);
  driveDistance(-275);
  setLiftTarget(800);
  followPath(7);
  setLiftTarget(700);
  wait(0.4, sec);
  claw.set(false);
  setLiftTarget(0);
  followPath(9, PathOpts().reverse());
  followPath(10);
  liftToHome(true, 400);
  claw.set(true);
  wait(0.4, sec);
  driveDistance(-300);
  setLiftTarget(20);
  setLiftTarget(110);
  turnToHeading(180);
  followPath(11);
  liftTo(40, 600);
  claw.set(false);
  driveDistance(-200);
  setLiftTarget(0);
  turnToHeading(180);
  followPath(12);
  liftToHome(true, 400);
  claw.set(true);
  wait(0.6, sec);
  setLiftTarget(10);
  driveDistance(-300);
  setLiftTarget(20);
  setLiftTarget(200);
  turnToHeading(180);
  followPath(11);
  claw.set(false);
  setLiftTarget(0);
  followPath(9, PathOpts().reverse());
  followPath(10);
  liftToHome(true, 400);
  claw.set(true);
  wait(0.4, sec);
  followPath(13, PathOpts().reverse().maxAccel(4000).maxVel(9000));
  setLiftTarget(220);
  turnToHeading(90);
  driveDistance(-400, 2000, 400);
  followPath(14, PathOpts().noStartTurn().maxAccel(4000).maxVel(9000));
  liftTo(0, 800);
  claw.set(false);
  endRunTimer();
  
}


void usercontrol(void) {
  setLiftControl(false);

  while (1) {
    setDrive();
    moveCascade();

    static bool prevR2 = false;
    bool curR2 = Controller.ButtonR2.pressing();
    if(curR2 && !prevR2) {
      toggleClaw();
    }
    prevR2 = curR2;

    wait(20, msec); 
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
