/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       jackj                                                     */
/*    Created:      9/2/2026, 6:38:16 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/


/*
  IMPORTANT STUFF (units: mm, mm/s, ms; field heading 0 = +x, 90 = +y, CCW +)

  Paths
    followPath(n);                         //path n from pathify.py (1-based)
    followPath(n, PathOpts()               //optional:
        .maxVel(800)                       //top speed 
        .endTol(30)                        //end tolerance
        .maxAccel(1200)                    //accel/decel limit
        .reverse()                         //drive backwards
        .noStartTurn());                   //skip the turn at the start

  Flags (add BEFORE the followPath they belong to; cleared after it finishes)
    void grab() { claw.set(true); }        //action = void function, no args
    addFlag({x, y}, grab);                 //fire when passing (x,y)
    addFlag({x, y}, grab, 150);            //fire 150 mm before (x,y)
    addStopFlag({x, y}, grab, 300);        //stop at (x,y), fire, wait 300 ms, continue
   

  Other stuff
    driveDistance(-300);                   //straight, holds heading, - = backwards
    driveDistance(500, 600);               //2nd arg = max speed
    turnToHeading(90);                     //absolute heading, shortest way
    turnToFace({x, y});                    //point the front at a field point
    setLiftTarget(mm);  liftTo(mm);        //non-blocking / blocking lift
*/

#include "vex.h"
#include "motorDefs.h"
#include "pursuit.h"
#include "lift.h"
#include "brainUI.h"

using namespace vex;



int main() {

  vex::thread UIThread(brainUI);
  claw.set(true);
  initOdom();
  vex::thread odomThread(updateOdom);
  startLift();   

 startRunTimer();
 roller.set(true);
 wait(0.3, sec);
 setLiftTarget(110);
 followPath (1);
 setLiftTarget(45);
 wait(0.3, sec);
 claw.set(false);
 driveDistance(-300);
 setLiftTarget(0);
 followPath(2);
 claw.set(true);
 wait(0.2, sec);
 setLiftTarget(205);
 followPath(3, PathOpts().endTol(60));
 claw.set(false);
 wait(0.2, sec);
 driveDistance(-300);
 setLiftTarget(0);
 followPath(4, PathOpts().endTol(60));
 claw.set(true);
 wait(0.3, sec);
 setLiftTarget(390);
 driveDistance(-500);
 followPath(5);
 claw.set(false);
 driveDistance(-100);
 setLiftTarget(0);
 followPath(6, PathOpts().reverse());
 followPath(7, PathOpts().noStartTurn());
 claw.set(true);
 setLiftTarget(440);
 followPath(8, PathOpts().noStartTurn().reverse());
 followPath(9);
 endRunTimer();
}
