#include "vex.h"
#include "motorDefs.h"
#include <string>

extern int motorStatusTimer;
extern bool frontLeftStatus;
extern bool midLeftStatus;
extern bool backLeftStatus;
extern bool frontRightStatus;
extern bool midRightStatus;
extern bool backRightStatus;
extern bool lift1Status;
extern bool lift2Status;
extern bool lift3Status;
extern bool odomStatus;
extern bool inertialStatus;
extern bool liftStatus;
extern bool calibrated;

void drawButton(int x, int y, int w, int h, std::string t, int destination);
void drawAutonButton(int x, int y, int w, int h, std::string t, int autonNum, vex::color textColor = vex::white);

void brainUI(void);
