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

vex::motor lift1(vex::PORT1, false);
vex::motor lift2(vex::PORT2, true);
vex::motor lift3(vex::PORT3, false);

int main() {

  initOdom();
  vex::thread odomThread(updateOdom);
  followPath();
  
}