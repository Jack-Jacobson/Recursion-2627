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
constexpr double lookaheadMm          = 180.0;                         
constexpr double waypointToleranceMm  = 20.0;                          
constexpr double turnKp               = 40.0;                          
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
constexpr double maxMotorRpm    = 600.0;   // ratio6_1 = blue cartridge

constexpr double maxVelMmS         = 1200.0;  // top speed 
constexpr double maxAccelMmS2      = 2000.0;  // accel + decel limit
constexpr double maxLatAccelMmS2   = 1500.0;  // Max corner speed before slowing
constexpr double endToleranceMm    = 25.0;  // Tolerence

constexpr double startTurnToleranceRad = 0.35;
constexpr double minTurnPct            = 8.0;
constexpr double controlLoopMs         = 20.0;

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

double distanceToPathEnd();

double targetVelocity(double kappa, double distLeft, double prevVel, double dt);

void driveWithCurvature(double v, double kappa);

void turnToFace(const Point& target);

void followPath();
