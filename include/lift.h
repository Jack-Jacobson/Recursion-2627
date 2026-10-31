#pragma once

#include "vex.h"
#include "motorDefs.h"

// Lift PID on the distance sensor heights are mm of travel, 0 = position at startLift()

constexpr double liftMaxHeightMm  = 1000.0; // targets clamped to max if this
constexpr double liftSensorMaxMm  = 1000.0; // readings above this are invalid

// Median then EMA
constexpr int    liftMedianSize   = 5;
constexpr double liftFilterAlpha  = 0.35;   // lower = smoother but laggier

constexpr double liftKp           = 0.125;
constexpr double liftKi           = 0.0005;
constexpr double liftKd           = 1.1;    // V per m/s
constexpr double liftKg           = 0.0;  
constexpr double liftMaxVolts     = 12.0;

constexpr double liftDownKpScale  = 0.5; // Lower for overshoot when going down
constexpr double liftMaxDownVolts = 6.0;
constexpr double liftDownKd       = 1.5;
constexpr double liftLagCompS     = 0.06;   // filter lag compensation
constexpr double liftMaxIntegral  = 3000.0;
constexpr double liftIZoneMm      = 40.0;

// Brake-hold inside Enter, resume PID past Exit (both above sensor noise).
constexpr double liftHoldEnterMm  = 6.0;
constexpr double liftHoldExitMm    = 14.0;

constexpr double liftToleranceMm  = 10.0;
constexpr int    liftSettleLoops  = 5;
constexpr int    liftLoopMs       = 10;

// Call once with the lift fully down
void startLift();

// Non-blocking
void setLiftTarget(double mm);

// false = lift thread stops commanding the motors (driver control drives them directly)
void setLiftControl(bool enabled);

// Blocking until settled or timeout
void liftTo(double mm, int timeoutMs = 1200);

double getLiftTarget();
double getLiftHeight();
bool   liftAtTarget();

// Encoder home (independent of distance sensor / lift PID)
constexpr double liftHomeVelocityPct = 100.0;

// Saves the current encoder position of lift1/2/3 as "home". Call at program start.
void recordLiftHome();

// Drives each lift motor back to its recorded encoder position using the motor's
// built-in position control. Overrides the lift PID and current target; the PID
// stays paused until the next setLiftTarget()/liftTo() call.
// wait = true blocks until there or timeout, false returns immediately.
void liftToHome(bool wait = true, int timeoutMs = 2000);
