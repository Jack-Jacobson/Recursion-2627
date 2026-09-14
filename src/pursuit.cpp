/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       pursuit.cpp                                               */
/*    Author:       jackj                                                     */
/*    Description:  Odometry + waypoint pursuit helpers                       */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "pursuit.h"

using namespace vex;

Pose pose;

Point path[] = {
    {0.0,   0.0},
    {300.0, 0.0},
    {300.0, 250.0},
    {700.0, 250.0}
};
extern const int pathLength = sizeof(path) / sizeof(path[0]);

int targetIndex = 0;

static double prevOdomDeg = 0.0;
static double prevYawDeg  = 0.0;


Point getTarget() {
    return path[targetIndex];
}

bool reachedTarget(const Point& target, const Pose& pose) {
    double dx = target.x - pose.x;
    double dy = target.y - pose.y;
    double distance = sqrt(dx * dx + dy * dy);
    return distance < waypointToleranceMm;
}



double normalizeAngle(double angle) {
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double headingErrorToTarget(const Point& target, const Pose& pose) {
    double targetAngle = atan2(target.y - pose.y, target.x - pose.x);
    return normalizeAngle(targetAngle - pose.theta);
}



void initOdom() {
    odomPod.resetPosition();
    this_thread::sleep_for(50);
    prevOdomDeg = odomPod.position(rotationUnits::deg);

    inertialSensor.calibrate();
    while (inertialSensor.isCalibrating()) {
        this_thread::sleep_for(10);
        Brain.Screen.print("Calibrating Inertial Sensor...");
    }
    prevYawDeg = inertialSensor.heading();
}

void updateOdom() {
    while (true) {
        double currOdomDeg = odomPod.position(rotationUnits::deg);
        double deltaDeg    = currOdomDeg - prevOdomDeg;
        double deltaMm     = (deltaDeg / 360.0) * wheelCircumferenceMm;

        double currYawDeg  = inertialSensor.heading();
        double deltaYawDeg = currYawDeg - prevYawDeg;
        if      (deltaYawDeg >  180) deltaYawDeg -= 360;
        else if (deltaYawDeg < -180) deltaYawDeg += 360;
        double deltaTheta  = deltaYawDeg * M_PI / 180.0;

        double midTheta = pose.theta + deltaTheta * 0.5;
        pose.x     += deltaMm * cos(midTheta);
        pose.y     += deltaMm * sin(midTheta);
        pose.theta += deltaTheta;

        prevOdomDeg = currOdomDeg;
        prevYawDeg  = currYawDeg;

        this_thread::sleep_for(odomLoopMs);
    }
}

void driveToTarget(const Pose& robot, const Point& target) {
    double error = headingErrorToTarget(target, robot);

    double turnPower    = error * turnKp;
    double forwardPower = forwardPowerPct;

    double left  = forwardPower - turnPower;
    double right = forwardPower + turnPower;

    frontLeftDrive.spin(directionType::fwd,  left,  velocityUnits::pct);
    midLeftDrive.spin(directionType::fwd,    left,  velocityUnits::pct);
    backLeftDrive.spin(directionType::fwd,   left,  velocityUnits::pct);

    frontRightDrive.spin(directionType::fwd, right, velocityUnits::pct);
    midRightDrive.spin(directionType::fwd,   right, velocityUnits::pct);
    backRightDrive.spin(directionType::fwd,  right, velocityUnits::pct);
}

void stopDrive() {
    frontLeftDrive.stop();
    midLeftDrive.stop();
    backLeftDrive.stop();
    frontRightDrive.stop();
    midRightDrive.stop();
    backRightDrive.stop();
}

static PathProgress progress;

bool findLookaheadPoint(const Pose& robot, double radius, Point& out) {
    for (int i = progress.segment; i < pathLength - 1; i++) {
        Point E = path[i]; //start of segment
        Point L = path[i + 1]; //end of segment

        double dx = L.x - E.x,       dy = L.y - E.y; //Segment direction vector
        double fx = E.x - robot.x,   fy = E.y - robot.y; //Vector from robot to segment start

        //Coeffecints for quadratic equation
        double a = dx*dx + dy*dy;
        double b = 2.0 * (fx*dx + fy*dy);
        double c = fx*fx + fy*fy - radius*radius;
        double disc = b*b - 4.0*a*c; //Discriminant (positive = intersection, negative = no intersection)

        if (a < 1e-9 || disc < 0.0) continue; //if no crossing

        disc = sqrt(disc); //Square root of discriminant

        //Solving the quadratic formula
        double t1 = (-b - disc) / (2.0*a); //entry point (behind)
        double t2 = (-b + disc) / (2.0*a); //exit point, preferred (ahead)

        //Ensure t is in range of the segmant and pick closest one
        double t = -1.0;
        if      (t2 >= 0.0 && t2 <= 1.0) t = t2;  
        else if (t1 >= 0.0 && t1 <= 1.0) t = t1;
        if (t < 0.0) continue; 

        if (i == progress.segment && t < progress.t) continue; //Ensure forward progress along the segmant

        //Update the progress and return the first valid lookahead point
        progress.segment = i; 
        progress.t       = t;
        out.x = E.x + t*dx;
        out.y = E.y + t*dy;
        return true;
    }
    return false; 
}

double curvatureTo(const Pose& robot, const Point& look) {
    double dx = look.x - robot.x;
    double dy = look.y - robot.y;

    double localY = -dx*sin(robot.theta) + dy*cos(robot.theta);
    double lenSq  = dx*dx + dy*dy;

    if (lenSq < 1e-9) return 0.0;
    return 2.0 * localY / lenSq;
}

static double mmsToPct(double mmPerSec) {
    double wheelRps = mmPerSec / (M_PI * driveWheelMm);
    double motorRpm = wheelRps * 60.0 / wheelPerMotor;
    return motorRpm / maxMotorRpm * 100.0;
}

void driveWithCurvature(double v, double kappa) {
    double offset = v * kappa * (trackWidthMm / 2.0);
    setDrive(mmsToPct(v - offset), mmsToPct(v + offset));
}