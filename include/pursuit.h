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


constexpr double wheelDiameterMm      = 50.8;                       
constexpr double wheelCircumferenceMm = M_PI * wheelDiameterMm;       
// Adaptive lookahead: L = clamp(v * lookaheadTimeS, minLookaheadMm, lookaheadMm). Short lookahead for slow speeds, longer for fast speeds.
constexpr double lookaheadMm          = 500.0; // max lookahead
constexpr double minLookaheadMm       = 200.0;
constexpr double lookaheadTimeS       = 0.45;
constexpr double curvatureGain        = 1.0;   // >1 if robot turns less than needed
constexpr double waypointToleranceMm  = 20.0;                          
constexpr double turnKp               = 1.0;
constexpr double forwardPowerPct      = 25.0;                          
constexpr double maxPowerPct          = 100.0;                         
constexpr int    odomLoopMs           = 10;                            

extern Pose pose;           
extern Point path[];        
extern const int pathLength; 
extern int targetIndex;     

constexpr double trackWidthMm   = 293.0144;   // LEFT CENTER TO RIGHT CENTER
constexpr double driveWheelMm   = 69.85;   // WHEEL DIAMATER 
constexpr double wheelPerMotor  = 36.0 / 48.0;    // WHEEL REVOLUTION BY MOTOR REVOLUTION
constexpr double maxMotorRpm    = 600.0;   

// fastest wheel speed in mm/s
constexpr double maxWheelVelMmS = (maxMotorRpm * wheelPerMotor / 60.0) * M_PI * driveWheelMm;

constexpr double maxVelMmS         = 1400.0;  // top speed (must stay under maxWheelVelMmS)
constexpr double maxAccelMmS2      = 1600.0;  // accel + decel limit
constexpr double maxLatAccelMmS2   = 2500.0;  // lateral acceleration limit (lower = less wheel slip)
constexpr double endToleranceMm    = 20.0;  // stop radius (the end-line check also stops it)
constexpr double cornerMaxAngleRad = 2.6;   // clamp corner angle used for corner speed planning (~150 deg)

static_assert(maxVelMmS < maxWheelVelMmS, "maxVelMmS exceeds what the drivetrain can reach; decel planning will overshoot");

constexpr double startTurnToleranceRad = 0.02;  // ~1.1 deg
constexpr int    startTurnSettleLoops  = 5;     // loops inside tolerance before accepting
constexpr int    maxTurnMs             = 2000;  // give up instead of oscillating forever
constexpr double minTurnPct            = 12.0;  // voltage mode: must overcome static friction
constexpr double turnToFaceKp          = 40.0;  // % power per radian of heading error
constexpr double turnToFaceKd          = 3.0;   // % power per rad/s of error change (damping)
constexpr double controlLoopMs         = 10.0;

// Voltage drive feedforward (per wheel side)
constexpr double maxDriveVolts = 12.0;
// volts = kS*sign(v) + kV*v + kA*a + kP*(v - measured)
constexpr double driveKs       = 0.04;                           // V
constexpr double driveKv       = 0.00628;                        // V per mm/s (from your test run)
constexpr double driveKa       = 0.0010;                         // V per mm/s^2
constexpr double driveKp       = 0.0020;                         // V per mm/s of error
constexpr double maxFeedforwardAccelMmS2 = 2.0 * maxAccelMmS2;   // clamp for kA term

// Yaw-rate feedback (gyro). Compares commanded turn rate (v * kappa) with the
// measured one and adds a left/right voltage difference to close the gap.
// Fixes under-turning on curves. Set both gains to 0 to disable.
constexpr double yawRateKp       = 2.0;   // V per side, per rad/s of turn-rate error
constexpr double yawRateKi       = 4.0;   // V per side, per rad of heading lag (removes constant pull)
constexpr double yawRateMaxVolts = 3.0;   // clamp on the correction
constexpr int    yawRateWindow   = 4;     // loops used to differentiate heading (40 ms)

// Bail out instead of pushing into a wall: if less than stallMinMoveMm of travel happens over stallCheckMs while commanding motion, stop.
constexpr double stallMinMoveMm = 10.0;
constexpr double stallCheckMs   = 600.0;

// starting heading of the robot in radians
constexpr double startHeadingRad = 0;

// print to console
constexpr bool debugTelemetry = true;

// how many control loops betweens console prints during followPath()
constexpr int debugPrintLoopInterval = 10;

// minimum drive speed when robot should move
constexpr double minVelocityMmS = 250.0;

struct PathProgress {
    int    segment = 0;    // INDEX
    double t       = 0.0;  // FRACTOIN
};


Pose getPose();

Point getTarget();

bool reachedTarget(const Point& target, const Pose& pose);

double normalizeAngle(double angle);


double headingErrorToTarget(const Point& target, const Pose& pose);


void initOdom();


void updateOdom();


void setDrive(double left, double right);

void setDriveVolts(double leftVolts, double rightVolts);

double lookaheadFor(double velMmS);

void driveToTarget(const Pose& robot, const Point& target);

void stopDrive();

bool findLookaheadPoint(const Pose& robot, double radius, Point& out);

double curvatureTo(const Pose& robot, const Point& look);

double distanceToPathEnd(const Pose& robot);

double targetVelocity(double kappa, double distLeft, double prevVel, double dt);

void driveWithCurvature(double v, double kappa);

void turnToFace(const Point& target);

void followPath();