
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

// Previous sensor readings, kept between odometry iterations.
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
        // How far the tracking wheel rolled since last loop.
        double currOdomDeg = odomPod.position(rotationUnits::deg);
        double deltaDeg    = currOdomDeg - prevOdomDeg;
        double deltaMm     = (deltaDeg / 360.0) * wheelCircumferenceMm;

        // How far the robot turned since last loop, wrapped across 0/360.
        double currYawDeg  = inertialSensor.heading();
        double deltaYawDeg = currYawDeg - prevYawDeg;
        if      (deltaYawDeg >  180) deltaYawDeg -= 360;
        else if (deltaYawDeg < -180) deltaYawDeg += 360;
        double deltaTheta  = deltaYawDeg * M_PI / 180.0;

        // Integrate the arc using the heading at the midpoint of the step.
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
