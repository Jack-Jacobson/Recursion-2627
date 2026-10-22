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
 liftTo(150);
 followPath(1);
 followPath(2);
 liftTo(35);
 claw.set(false);
 wait(0.5, sec);
 driveDistance(-500);
 setLiftTarget(0);
 followPath(3);
 driveDistance(80);
 claw.set(true);
 wait(0.3, sec);
 driveDistance(-120);
 setLiftTarget(400);
 followPath(4);
 liftTo(200);
 claw.set(false);
 wait(0.3 , sec);
 driveDistance(-300);
 //followPath(5);
 endRunTimer();
}
