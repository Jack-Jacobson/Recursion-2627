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
constexpr double lookaheadMm          = 400.0;
constexpr double minLookaheadMm       = 120.0;
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
constexpr double maxAccelMmS2      = 1200.0;  // accel + decel limit
constexpr double maxLatAccelMmS2   = 3000.0;  // lateral acceleration limit
constexpr double endToleranceMm    = 50.0;  // Tolerence
constexpr double cornerMaxAngleRad = 2.6;   // clamp corner angle used for corner speed planning (~150 deg)

static_assert(maxVelMmS < maxWheelVelMmS,
              "maxVelMmS exceeds what the drivetrain can reach; decel planning will overshoot");

constexpr double startTurnToleranceRad = 0.05;  // ~3 deg
constexpr int    startTurnSettleLoops  = 5;     // loops inside tolerance before accepting
constexpr int    maxTurnMs             = 2000;  // give up instead of oscillating forever
constexpr double minTurnPct            = 6.0;   // must be > 0 or turnToFace can hang
constexpr double turnToFaceKp          = 40.0;  // % power per radian of heading error
constexpr double controlLoopMs         = 10.0;

// Bail out instead of pushing into a wall: if less than stallMinMoveMm of
// travel happens over stallCheckMs while we are commanding motion, stop.
constexpr double stallMinMoveMm = 10.0;
constexpr double stallCheckMs   = 300.0;

// starting heading of the robot in radians
constexpr double startHeadingRad = 0;

// print to console
constexpr bool debugTelemetry = true;

// how many control loops betweens console prints during followPath()
constexpr int debugPrintLoopInterval = 2;

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

void driveToTarget(const Pose& robot, const Point& target);

void stopDrive();


bool findLookaheadPoint(const Pose& robot, double radius, Point& out);

double curvatureTo(const Pose& robot, const Point& look);

double distanceToPathEnd(const Pose& robot);

double targetVelocity(double kappa, double distLeft, double prevVel, double dt);

void driveWithCurvature(double v, double kappa);

void turnToFace(const Point& target);

void followPath();