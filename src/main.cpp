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
#include "pursuit.h"

using namespace vex;

int main() {
    initOdom();

    vex::thread odomThread(updateOdom);

    while (true) {
        Point target = getTarget();

        if (reachedTarget(target, pose)) {
            targetIndex++;
            if (targetIndex >= pathLength) {
                stopDrive();
                break;
            }
            continue;
        }

        driveToTarget(pose, target);

        this_thread::sleep_for(20);
    }
}
