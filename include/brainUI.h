#include "vex.h"
#include "motorDefs.h"

extern int motorStatusTimer = 0;
extern bool frontLeftStatus = frontLeftDrive.installed();
extern bool midLeftStatus = midLeftDrive.installed();
extern bool backLeftStatus = backLeftDrive.installed();
extern bool frontRightStatus = frontRightDrive.installed();
extern bool midRightStatus = midRightDrive.installed();
extern bool backRightStatus = backRightDrive.installed();
extern bool lift1Status = lift1.installed();
extern bool lift2Status = lift2.installed();
extern bool lift3Status = lift3.installed();
extern bool odomStatus = odomPod.installed();
extern bool intertialStatus = inertialSensor.installed();
extern bool liftStatus = liftSensor.installed();
extern bool calibrated = false;

void drawButton(int x, int y, int w, int h, std::string t, int destination);
void drawAutonButton(int x, int y, int w, int h, std::string t, int autonNum, color textColor = white);

void brainUI(void);
