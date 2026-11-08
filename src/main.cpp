/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       jackj                                                     */
/*    Created:      10/22/2026, 3:40:26 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

/*
================================ CHEAT SHEET ================================
Units: mm, mm/s, degrees unless noted. Heading 0 = +x, 90 = +y, CCW +.

--- PATH FOLLOWING ---------------------------------------------------------
followPath(n, opts)          Follow generated path n (1-based)
                             [opts = PathOpts() (all defaults below)]
followPath(pts, len, opts)   Follow a custom Point array  [opts = PathOpts()]
driveDistance(dist, maxVel, stallMs, maxAccel)
                             Straight move holding heading; negative = back up
                             [maxVel=800 mm/s, stallMs=220, maxAccel=1600 mm/s^2]
turnToFace(Point, tolRad)    Turn to face a field point  [tolRad=0.02 (~1.1 deg)]
turnToHeading(deg, tolRad)   Turn to absolute field heading (shortest way)
                             [tolRad=0.02 (~1.1 deg)]
setPathMirror(bool)          Mirror paths/flags/turns across y = x  [off]
resetPoseToStart()           Reset odom to start of path 1 (call after mirror)
getPose()                    Current x, y, theta
startRunTimer() / endRunTimer()   Time the run

--- PathOpts() (chain after followPath) -----------------------------------
.maxVel(v)           Top speed                         [1400 mm/s, cap ~1560]
.maxAccel(a)         Accel + decel limit               [1600 mm/s^2]
.maxLatAccel(a)      Corner speed limit; higher = faster turns, more slip
                                                       [2500 mm/s^2]
.endTol(mm)          Distance from end that counts as done  [20 mm]
.endVel(v)           Speed allowed at end of path      [0 = stop]
.stallMs(ms)         End path if barely moving this long    [220 ms]
.stallLastPts(n)     Only stall-check in last n points      [0 = always]
.reverse(r)          Drive path backwards (back leads) [off; r=true if called]
.noStartTurn()       Skip initial turn toward path     [start turn ON]
.startTurnTol(deg)   Start turn done within this error [~2.0 deg (0.035 rad)]
.startTurnAccel(a)   Start turn ramp-up rate           [14 rad/s^2]
.grabAt(mm)          Close claw + end path when clawDist <= mm
                                                       [off; mm=50 if called]
.grabLift(mm)        Set lift target to mm when grab triggers  [off]

--- EVENTS / FLAGS (fire an action at a point on the path) -----------------
addFlag(Point, func, leadMm)             Run void func() near Point; fires leadMm early
                                         [leadMm=0]
addStopFlag(Point, func, waitMs, leadMm) Stop at Point, run func, hold waitMs, continue
                                         [leadMm=0; waitMs required]
clearFlags()                             Remove all flags (max 16)

--- LIFT -------------------------------------------------------------------
startLift()               Zero lift (call once, lift fully down)
setLiftTarget(mm)         Non-blocking height target  [clamped to 0..1000]
liftTo(mm, timeoutMs)     Blocking until settled      [timeoutMs=1200]
setLiftControl(bool)      false = driver drives lift directly
getLiftTarget() / getLiftHeight() / liftAtTarget()   Lift status
recordLiftHome()          Save lift motor encoder positions as home
liftToHome(wait, timeoutMs)  Return to saved home     [wait=true, timeoutMs=2000]

--- OTHER ------------------------------------------------------------------
claw.set(true/false)      Claw state (true = closed)
roller.set(true/false)    Roller on/off
toggleClaw()              Flip claw
rightToggle()/leftToggle()  Driver toggles (B / Down)
wait(x, sec/msec)         Delay
currentAuton              Brain UI auton selection

--- DRIVER BUTTONS ---------------------------------------------------------
L2 / L1 held   Cascade up / down
R2             Toggle claw
R1 held        Auto-claw when object <= autoClawMm  [45 mm]
B / Down       rightToggle / leftToggle
Rumble         At 60 s, 30 s, 10 s left
=============================================================================
*/

#include "vex.h"
#include "drive.h"
#include "lift.h"
#include "brainUI.h"
#include "pursuit.h"

using namespace vex;

competition Competition;

void pre_auton(void)
{
  vex::thread UIThread(brainUI);
  claw.set(true);
  initOdom();
  vex::thread odomThread(updateOdom);
  startLift();
}

void scorePin(void)
{
  startRunTimer();
  roller.set(true);
  wait(0.3, sec);
  setLiftTarget(110);
  followPath(1, PathOpts().noStartTurn());
  liftTo(60);
  wait(0.3, sec);
  claw.set(false);
  driveDistance(-400, 800, 300);
  setLiftTarget(-20);
}

static volatile bool autoHasRun = false;
static volatile bool driverAutoDone = false;

void autonomous(void)
{
  autoHasRun = true;
  /**
  // 1 = Non-Loader: same routine mirrored onto the other side.
  setPathMirror(currentAuton == 1);
  resetPoseToStart();

  switch (currentAuton) {
    case 0:
    case 1:
      loaderToggleAuton();
      break;
    case 2:
      scorePin();
      break;
    default:
      driveDistance(90);
      break;
  }
  */
  startRunTimer();
  roller.set(true);
  wait(0.3, sec);
  setLiftTarget(128);
  followPath(1, PathOpts().noStartTurn().maxAccel(2600).stallLastPts(6).stallMs(50));
  setLiftTarget(0);
  wait(0.25, sec);
  claw.set(false);
  driveDistance(-300, 3000, 220, 2600);
  setLiftTarget(139);
  followPath(2, PathOpts().startTurnTol(5).startTurnAccel(30).grabAt(110).grabLift(200).endVel(500).grabLift(300));
  addFlag(path3[path3Length / 2], []() { setLiftTarget(190); });
  followPath(3, PathOpts().noStartTurn());
  setLiftTarget(0);
  wait(0.2, sec);
  claw.set(false);
  followPath(4, PathOpts().reverse().maxAccel(6000).stallLastPts(6).stallMs(50));
  followPath(5, PathOpts().endVel(600).grabAt(120).grabLift(270).noStartTurn());
  claw.set(true);
  setLiftTarget(270);
  followPath(6, PathOpts().noStartTurn().endVel(600));
  liftTo(0, 400);
  claw.set(false);
  driveDistance(-300, 4000, 220, 3500);
  setLiftTarget(0);
  followPath(7, PathOpts().endVel(600).grabAt(155).maxAccel(2900).grabLift(270).startTurnAccel(35).endVel(400));
  claw.set(true);
  setLiftTarget(270);
  followPath(8, PathOpts().maxAccel(2600));
  liftTo(0, 400);
  claw.set(false);
  endRunTimer();
}

constexpr double driverPeriodSec = 105.0;

// Auto claw trigger distance (mm) while R1 is held.
double autoClawMm = 45.0;
constexpr double autoClawHeadwayS = 0.06;
constexpr double autoClawMaxMm = 130.0;
extern bool clawed;


struct RumbleCue
{
  double secLeft;
  const char *pattern; // '.' short, '-' long, ' ' pause
  bool done;
};

static int driverAutoTask()
{
  autonomous();
  driverAutoDone = true;
  return 0;
}
void usercontrol(void) {
  roller.set(true);

  RumbleCue cues[] = {
    {60.0, "-",     false},
    {30.0, "- -",   false},
    {10.0, "-.-", false},
    
  };
  vex::timer driverTimer;

  // Run the auto at the start of driver control unless it already ran in the
  // autonomous period. X cancels it and hands control to the driver.
  vex::thread *autoThread = nullptr;
  if (!autoHasRun) {
    autoThread = new vex::thread(driverAutoTask);
  } else {
    setLiftControl(false);
  }

  while (1) {
    if (autoThread != nullptr) {
      if (Controller.ButtonX.pressing() || driverAutoDone) {
        if (!driverAutoDone) {
          autoThread->interrupt();
          clearFlags();
          stopDrive();
        }
        setLiftControl(false);
        autoThread = nullptr;
      } else {
        wait(20, msec);
        continue;
      }
    }

    setDrive();
    moveCascade();

    double secLeft = driverPeriodSec - driverTimer.time(sec);
    for (RumbleCue& c : cues) {
      if (!c.done && secLeft <= c.secLeft) {
        Controller.rumble(c.pattern);
        c.done = true;
      }
    }

    static bool prevR2 = false;
    bool curR2 = Controller.ButtonR2.pressing();
    if(curR2 && !prevR2) {
      toggleClaw();
    }
    prevR2 = curR2;

    if (Controller.ButtonR1.pressing() && !clawed && clawDist.installed()) {
      double dynamicClawMm = fmin(autoClawMaxMm, autoClawMm + getOdomSpeedMmS() * autoClawHeadwayS);
      double objMm = clawDist.objectDistance(mm);
      if (objMm > 0.0 && objMm <= dynamicClawMm) {
        clawed = true;
        claw.set(true);
      }
    }

    static bool prevRight = false;
    bool curRight = Controller.ButtonRight.pressing();
    if (curRight && !prevRight) {
      setLiftControl(true);
      setLiftTarget(138);
    }
    prevRight = curRight;

    static bool prevY = false;
    bool curY = Controller.ButtonY.pressing();
    if (curY && !prevY && !clawed && clawDist.installed()) {
      double dynamicClawMm = fmin(autoClawMaxMm, autoClawMm + getOdomSpeedMmS() * autoClawHeadwayS);
      double objMm = clawDist.objectDistance(mm);
      if (objMm > 0.0 && objMm <= dynamicClawMm) {
        clawed = true;
        claw.set(true);
        setLiftControl(true);
        setLiftTarget(30);
      }
    }
    prevY = curY;

    wait(20, msec); 
  

    static bool prevB = false;
    bool curB = Controller.ButtonB.pressing();
    if (curB && !prevB){
      rightToggle();
    }
    prevB = curB;

    static bool prevDown = false;
    bool curDown = Controller.ButtonDown.pressing();
    if (curDown && !prevDown){
      leftToggle();
    }
    prevDown = curDown;

  }

  wait(20, msec);
}
//
// Main will set up the competition functions and callbacks.
//
int main()
{
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true)
  {
    wait(100, msec);
  }
}
