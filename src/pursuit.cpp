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

static vex::mutex poseMutex;


Pose getPose() {
    poseMutex.lock();
    Pose snapshot = pose;
    poseMutex.unlock();
    return snapshot;
}

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



static double inertialThetaRad() {
    return -inertialSensor.rotation(rotationUnits::deg) * M_PI / 180.0;
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

    poseMutex.lock();
    pose.x = 0;
    pose.y = 0;
    pose.theta = inertialThetaRad();
    poseMutex.unlock();
}

void updateOdom() {
    while (true) {
        double currOdomDeg = odomPod.position(rotationUnits::deg);
        double deltaDeg    = currOdomDeg - prevOdomDeg;
        double deltaMm     = (deltaDeg / 360.0) * wheelCircumferenceMm;
        prevOdomDeg        = currOdomDeg;

        double currTheta = inertialThetaRad();

        poseMutex.lock();
        double deltaTheta = currTheta - pose.theta;

        double midTheta = pose.theta + deltaTheta * 0.5;
        pose.x    += deltaMm * cos(midTheta);
        pose.y    += deltaMm * sin(midTheta);
        pose.theta = currTheta;
        poseMutex.unlock();

        this_thread::sleep_for(odomLoopMs);
    }
}

void setDrive(double left, double right) {

    double peak = fmax(fabs(left), fabs(right));
    if (peak > maxPowerPct) {
        double scale = maxPowerPct / peak;
        left  *= scale;
        right *= scale;
    }

    frontLeftDrive.spin(directionType::fwd,  left,  velocityUnits::pct);
    midLeftDrive.spin(directionType::fwd,    left,  velocityUnits::pct);
    backLeftDrive.spin(directionType::fwd,   left,  velocityUnits::pct);

    frontRightDrive.spin(directionType::fwd, right, velocityUnits::pct);
    midRightDrive.spin(directionType::fwd,   right, velocityUnits::pct);
    backRightDrive.spin(directionType::fwd,  right, velocityUnits::pct);
}

void driveToTarget(const Pose& robot, const Point& target) {
    double error = headingErrorToTarget(target, robot);

    double turnPower = error * turnKp;

    setDrive(forwardPowerPct - turnPower, forwardPowerPct + turnPower);
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

double distanceToPathEnd() {
    Point A = path[progress.segment];
    Point B = path[progress.segment + 1];

    double dx = B.x - A.x;
    double dy = B.y - A.y;
    double total = sqrt(dx*dx + dy*dy) * (1.0 - progress.t);

    for (int i = progress.segment + 1; i < pathLength - 1; i++) {
        double sx = path[i + 1].x - path[i].x;
        double sy = path[i + 1].y - path[i].y;
        total += sqrt(sx*sx + sy*sy);
    }
    return total;
}

double targetVelocity(double kappa, double distLeft, double prevVel, double dt) {
    double v = maxVelMmS;

    if (fabs(kappa) > 1e-6)
        v = fmin(v, sqrt(maxLatAccelMmS2 / fabs(kappa)));

    v = fmin(v, sqrt(2.0 * maxAccelMmS2 * fmax(distLeft, 0.0)));
    v = fmin(v, prevVel + maxAccelMmS2 * dt);

    return fmax(v, 0.0);
}

void turnToFace(const Point& target) {
    while (true) {
        Pose robot = getPose();
        double error = headingErrorToTarget(target, robot);
        if (fabs(error) < startTurnToleranceRad) break;

        double power = error * turnKp;
        if (fabs(power) < minTurnPct) power = (power > 0.0) ? minTurnPct : -minTurnPct;

        setDrive(-power, power);
        this_thread::sleep_for(10);
    }
    stopDrive();
}

void followPath() {
    progress = PathProgress();

    turnToFace(path[1]);

    double vel = 0.0;
    const double dt = controlLoopMs / 1000.0;

    while (true) {
        Pose   robot = getPose();
        Point  look;
        double kappa;
        double distLeft;

        if (findLookaheadPoint(robot, lookaheadMm, look)) {
            kappa    = curvatureTo(robot, look);
            distLeft = distanceToPathEnd();
        } else {
            Point endPt = path[pathLength - 1];
            double dx = endPt.x - robot.x;
            double dy = endPt.y - robot.y;
            distLeft = sqrt(dx*dx + dy*dy);

            if (distLeft < endToleranceMm) break;

            kappa = curvatureTo(robot, endPt);
        }

        vel = targetVelocity(kappa, distLeft, vel, dt);
        driveWithCurvature(vel, kappa);

        this_thread::sleep_for((int)controlLoopMs);
    }

    stopDrive();
}
