#pragma once

#include "vex.h"
#include "motorDefs.h"

// Lift PID on the distance sensor heights are mm of travel, 0 = position at startLift()

constexpr double liftMaxHeightMm  = 430.0; // targets clamped to max if this
constexpr double liftSensorMaxMm  = 450.0; // readings above this are invalid

// Median then EMA
constexpr int    liftMedianSize   = 5;
constexpr double liftFilterAlpha  = 0.35;   // lower = smoother but laggier

constexpr double liftKp           = 0.08;
constexpr double liftKi           = 0.0005;
constexpr double liftKd           = 0.8;    // V per m/s
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
constexpr double liftHoldExitMm   = 14.0;

constexpr double liftToleranceMm  = 10.0;
constexpr int    liftSettleLoops  = 5;
constexpr int    liftLoopMs       = 10;

// Call once with the lift fully down
void startLift();

// Non-blocking
void setLiftTarget(double mm);

// Blocking until settled or timeout
void liftTo(double mm, int timeoutMs = 1200);

double getLiftTarget();
double getLiftHeight();
bool   liftAtTarget();
