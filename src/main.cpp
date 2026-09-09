/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       jackj                                                     */
/*    Created:      9/2/2026, 6:38:16 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "motorDefs.h"


using namespace vex;

struct Pose { double x = 0, y = 0, theta = 0; };
static Pose pose;
struct Point {
    double x;
    double y;
};
Point path[] = {
    {0.0, 0.0},
    {300.0, 0.0},
    {300.0, 250.0},
    {700.0, 250.0}
};
int targetIndex = 0;

Point getTarget(){
    return path[targetIndex];
}

bool reachedTarget(const Point& target, const Pose& pose) {
    double distance = sqrt(pow(target.x - pose.x, 2) + pow(target.y - pose.y, 2));
    return distance < 20.0; 
}

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}
double headingErrorToTarget(const Point& target, const Pose& pose) {
    double targetAngle = atan2(target.y - pose.y, target.x - pose.x);
    double error = normalizeAngle(targetAngle - pose.theta);
    return error;
}

const double lookaheadMm = 180.0;
const double wheelDiameterMm = 50.8;
const double wheelCircumferenceMm = M_PI * wheelDiameterMm;
double prevOdomDeg = 0.0;
double prevYawDeg = 0.0;

void updateOdom() {
    while(true) {
        double currOdomDeg = odomPod.position(rotationUnits::deg);
        double deltaDeg = currOdomDeg - prevOdomDeg;
        double deltaMm = (deltaDeg / 360.0) * wheelCircumferenceMm;

        double currYawDeg = inertialSensor.heading();
        double deltaYawDeg = currYawDeg - prevYawDeg;
        if(deltaYawDeg > 180) deltaYawDeg -= 360;
        else if(deltaYawDeg < -180) deltaYawDeg += 360;
        double deltaTheta = deltaYawDeg * M_PI / 180.0;

        double midTheta = pose.theta + deltaTheta * 0.5;
        pose.x += deltaMm * cos(midTheta);
        pose.y += deltaMm * sin(midTheta);
        pose.theta += deltaTheta;

        prevOdomDeg = currOdomDeg;
        prevYawDeg = currYawDeg;

        this_thread::sleep_for(10);
    }
}

void driveToTarget(const Pose& robot, const Point& target) {
    double error = headingErrorToTarget(target, robot);

    double turnPower = error * 2.5;  
    double forwardPower = 25.0;     
    double left = forwardPower - turnPower;
    double right = forwardPower + turnPower;

    frontLeftDrive.spin(vex::directionType::fwd, left, vex::velocityUnits::pct);
    midLeftDrive.spin(vex::directionType::fwd, left, vex::velocityUnits::pct);
    backLeftDrive.spin(vex::directionType::fwd, left, vex::velocityUnits::pct);

    frontRightDrive.spin(vex::directionType::fwd, right, vex::velocityUnits::pct);
    midRightDrive.spin(vex::directionType::fwd, right, vex::velocityUnits::pct);
    backRightDrive.spin(vex::directionType::fwd, right, vex::velocityUnits::pct);
}




int main() {

    odomPod.resetPosition();
    this_thread::sleep_for(50);
    prevOdomDeg = odomPod.position(rotationUnits::deg);

    inertialSensor.calibrate();
    while(inertialSensor.isCalibrating()){
        this_thread::sleep_for(10);
        Brain.Screen.print("Calibrating Inertial Sensor...");
    }
    prevYawDeg = inertialSensor.heading();

    vex::thread odomThread(updateOdom);

    while (true) {
    Point target = getTarget();

    if (reachedTarget(target, pose)) {
        targetIndex++;
        if (targetIndex >= 4) {
            frontLeftDrive.stop();
            midLeftDrive.stop();
            backLeftDrive.stop();
            frontRightDrive.stop();
            midRightDrive.stop();
            backRightDrive.stop();
            break;
        }
        continue;
    }

    driveToTarget(pose, target);

    this_thread::sleep_for(20);
}
}
