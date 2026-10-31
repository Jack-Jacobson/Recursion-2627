#pragma once

#include "vex.h"
#include "motorDefs.h"

// Driver cascade voltages (L2 held = up, L1 held = down)
constexpr double cascadeUpVolts   = 12.0;
constexpr double cascadeDownVolts = 12.0;

extern double Pgain[5];
extern double Igain[5];
extern double Dgain[5];

extern double allPastError[5];
extern double lastError[5];
extern double lastTimeRan[5];

void setDrive();
void moveCascade();
void toggleClaw();

double clamp(double value, double hiClamp, double loClamp, double deadHi, double deadLo);
double PID(double Ti, double error, int Values);
