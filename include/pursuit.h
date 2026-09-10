#pragma once

#include "vex.h"
#include "motorDefs.h"


struct Pose {
    double x = 0;
    double y = 0;
    double theta = 0;
};

struct Point {
    double x;
    double y;
};


constexpr double wheelDiameterMm      = 50.8;                          // tracking wheel diameter
constexpr double wheelCircumferenceMm = M_PI * wheelDiameterMm;        // mm travelled per full rotation
constexpr double lookaheadMm          = 180.0;                         // pure-pursuit lookahead radius (not used yet)
constexpr double waypointToleranceMm  = 20.0;                          // "close enough" radius for a waypoint
constexpr double turnKp               = 2.5;                           // P gain: radians of error -> motor percent
constexpr double forwardPowerPct      = 25.0;                          // base forward speed
constexpr int    odomLoopMs           = 10;                            // odometry update period

extern Pose pose;            // updated by updateOdom() on its own thread
extern Point path[];         // the waypoints to follow
extern const int pathLength; // number of waypoints in path[]
extern int targetIndex;      // which waypoint we're currently chasing

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Current waypoint we are driving toward.
Point getTarget();

// True once the robot is within waypointToleranceMm of the target.
bool reachedTarget(const Point& target, const Pose& pose);

// Wrap any angle (radians) into the range (-PI, PI].
double normalizeAngle(double angle);

// Signed angle (radians) the robot must turn to face the target.
double headingErrorToTarget(const Point& target, const Pose& pose);

// Reset the sensors and prime the odometry deltas. Call once before starting
// the odometry thread.
void initOdom();

// Infinite loop: integrates encoder + inertial readings into `pose`.
// Intended to be run on its own vex::thread.
void updateOdom();

// One iteration of the drive controller: point-and-shoot toward `target`.
void driveToTarget(const Pose& robot, const Point& target);

// Cut power to all six drive motors.
void stopDrive();
