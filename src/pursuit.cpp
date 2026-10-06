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
    {0, 0},
    {1000, 0},
    {1000, -800}
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
    pose.x     = path[0].x;
    pose.y     = path[0].y;
    pose.theta = normalizeAngle(startHeadingRad + inertialThetaRad());
    poseMutex.unlock();

    if (debugTelemetry) {
        printf("init pose x:%.1f y:%.1f hdg:%.1f\n",
               pose.x, pose.y, pose.theta * 180.0 / M_PI);
        fflush(stdout);
    }
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
static Point lastLook;
static bool  haveLastLook = false;

// Fraction along segment i of the robot's perpendicular projection (unclamped).
static double projectOnSegment(const Pose& robot, int i) {
    double dx = path[i + 1].x - path[i].x;
    double dy = path[i + 1].y - path[i].y;
    double lenSq = dx*dx + dy*dy;
    if (lenSq < 1e-9) return 1.0;
    return ((robot.x - path[i].x) * dx + (robot.y - path[i].y) * dy) / lenSq;
}

static double segmentLength(int i) {
    double dx = path[i + 1].x - path[i].x;
    double dy = path[i + 1].y - path[i].y;
    return sqrt(dx*dx + dy*dy);
}

// Picks the furthest-along intersection of the lookahead circle with the path
// that is not behind the previous one. If none exists (odom jitter, robot off
// the path) the previous lookahead point is reused instead of jumping to some
// other target, which is what made the steering flip back and forth.
bool findLookaheadPoint(const Pose& robot, double radius, Point& out) {
    bool found = false;
    PathProgress best = progress;
    Point bestPt = {0, 0};

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

        // The last segment is extended past the end point for STEERING only,
        // so the robot drives straight into the end instead of spiraling.
        // followPath() stops once the robot crosses the end line.
        double tMax = (i == pathLength - 2) ? 1e9 : 1.0;
        double t = -1.0;
        if      (t2 >= 0.0 && t2 <= tMax) t = t2;
        else if (t1 >= 0.0 && t1 <= tMax) t = t1;
        if (t < 0.0) continue;

        // Never move backwards along the segment; hold the previous point
        // instead of rejecting it (rejecting caused the target to flicker).
        if (i == progress.segment && t < progress.t) t = progress.t;

        // Keep looping: later segments are further along the path.
        found = true;
        best.segment = i;
        best.t       = t;
        bestPt.x = E.x + t*dx;
        bestPt.y = E.y + t*dy;
    }

    if (found) {
        progress     = best;
        lastLook     = bestPt;
        haveLastLook = true;
        out = bestPt;
        return true;
    }
    if (haveLastLook) {
        out = lastLook;
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

// Distance left along the path, measured straight to the end of the current
// segment (NOT from the lookahead point, which is lookaheadMm ahead). Using
// the true distance to the segment end instead of a clamped projection keeps
// the value honest when the robot has drifted off the path - a clamped
// projection reports 0 remaining while the robot is still far away, which
// makes the deceleration planner stop short of tolerance.
static int robotSegment = 0;

double distanceToPathEnd(const Pose& robot) {
    int seg = robotSegment;
    double total;
    if (seg == pathLength - 2) {
        // On the last segment measure along it, so the decel profile ends at
        // the end line (where followPath stops) even with some lateral error.
        total = segmentLength(seg) * (1.0 - projectOnSegment(robot, seg));
    } else {
        Point B = path[seg + 1];
        double dx = B.x - robot.x;
        double dy = B.y - robot.y;
        total = sqrt(dx*dx + dy*dy);
    }

    for (int i = seg + 1; i < pathLength - 1; i++) {
        double sx = path[i + 1].x - path[i].x;
        double sy = path[i + 1].y - path[i].y;
        total += sqrt(sx*sx + sy*sy);
    }
    return total;
}

// Advance the segment the robot is physically on (not where the lookahead is).
static double distToSegment(const Pose& robot, int i) {
    double t = fmin(fmax(projectOnSegment(robot, i), 0.0), 1.0);
    double px = path[i].x + t * (path[i + 1].x - path[i].x) - robot.x;
    double py = path[i].y + t * (path[i + 1].y - path[i].y) - robot.y;
    return sqrt(px*px + py*py);
}

// Pure pursuit cuts corners, so the robot may never project past the end of
// a segment. Also advance once it is closer to the next segment.
static void updateRobotSegment(const Pose& robot) {
    while (robotSegment < pathLength - 2) {
        int n = robotSegment + 1;
        bool past   = projectOnSegment(robot, robotSegment) >= 1.0;
        bool closer = projectOnSegment(robot, n) > 0.0 &&
                      distToSegment(robot, n) < distToSegment(robot, robotSegment);
        if (!past && !closer) break;
        robotSegment = n;
    }
}

// Speed limit from upcoming corners. Pure pursuit only sees a corner once the
// lookahead wraps it, which is too late to brake from full speed, so the
// robot used to overshoot and then S-curve back. Instead, estimate the arc
// pure pursuit will cut through each corner (tangent arc starting about half
// a lookahead before it) and start braking early enough to reach that speed.
double cornerSpeedCap(const Pose& robot) {
    double cap = maxVelMmS;

    double t = fmin(fmax(projectOnSegment(robot, robotSegment), 0.0), 1.0);
    double distToCorner = segmentLength(robotSegment) * (1.0 - t);

    for (int j = robotSegment + 1; j < pathLength - 1; j++) {
        double ax = path[j].x - path[j - 1].x, ay = path[j].y - path[j - 1].y;
        double bx = path[j + 1].x - path[j].x, by = path[j + 1].y - path[j].y;
        double turn = fabs(normalizeAngle(atan2(by, bx) - atan2(ay, ax)));
        turn = fmin(turn, cornerMaxAngleRad);

        double kCorner = 2.0 * tan(turn / 2.0) / lookaheadMm;
        double vCorner = maxVelMmS;
        if (kCorner > 1e-6) {
            vCorner = sqrt(maxLatAccelMmS2 / kCorner);
            vCorner = fmin(vCorner, maxWheelVelMmS / (1.0 + kCorner * trackWidthMm / 2.0));
        }
        vCorner = fmax(vCorner, minVelocityMmS);

        cap = fmin(cap, sqrt(vCorner*vCorner + 2.0 * maxAccelMmS2 * distToCorner));
        distToCorner += segmentLength(j);
    }
    return cap;
}

double targetVelocity(double kappa, double distLeft, double prevVel, double dt) {
    double v = maxVelMmS;

    double curveCap = maxVelMmS;
    if (fabs(kappa) > 1e-6)
        curveCap = sqrt(maxLatAccelMmS2 / fabs(kappa));

    // The outer wheel runs at v * (1 + |kappa| * trackWidth/2). If that
    // exceeds what the motors can do, setDrive scales both sides down and the
    // robot silently tracks a different curvature than planned.
    double outerRatio = 1.0 + fabs(kappa) * (trackWidthMm / 2.0);
    curveCap = fmin(curveCap, maxWheelVelMmS / outerRatio);

    v = fmin(v, curveCap);

    double decelCap = sqrt(2.0 * maxAccelMmS2 * fmax(distLeft, 0.0));
    v = fmin(v, decelCap);

    v = fmin(v, prevVel + maxAccelMmS2 * dt);

    // Don't let the accel ramp linger at a speed too low to actually move
    // the robot. Cap the floor at BOTH decelCap and curveCap so this never
    // overrides slowing down for a tight corner or stopping cleanly at the
    // end - it only helps the "haven't ramped up from a stop yet" case.
    if (v > 0.0 && v < minVelocityMmS) {
        v = fmin(minVelocityMmS, fmin(curveCap, decelCap));
    }

    return fmax(v, 0.0);
}

void turnToFace(const Point& target) {
    int settled = 0;
    int elapsedMs = 0;

    while (elapsedMs < maxTurnMs) {
        Pose robot = getPose();
        double error = headingErrorToTarget(target, robot);

        if (debugTelemetry) {
            printf("turn hdg:%.1f err:%.1f\n",
                   robot.theta * 180.0 / M_PI, error * 180.0 / M_PI);
            fflush(stdout);
        }

        // Require the error to stay inside tolerance for a few loops so we
        // don't accept a reading taken while coasting through the target.
        if (fabs(error) < startTurnToleranceRad) {
            if (++settled >= startTurnSettleLoops) break;
        } else {
            settled = 0;
        }

        double power = error * turnToFaceKp;
        if (fabs(power) < minTurnPct) power = (power > 0.0) ? minTurnPct : -minTurnPct;

        setDrive(-power, power);
        this_thread::sleep_for((int)controlLoopMs);
        elapsedMs += (int)controlLoopMs;
    }
    stopDrive();
    this_thread::sleep_for(100);
}

void followPath() {
    progress     = PathProgress();
    haveLastLook = false;
    robotSegment = 0;

    turnToFace(path[1]);

    double vel = 0.0;
    const double dt = controlLoopMs / 1000.0;
    int debugCounter = 0;

    const Point endPt = path[pathLength - 1];

    Pose stallRef = getPose();
    double stallTimerMs = 0.0;

    while (true) {
        Pose   robot = getPose();
        Point  look;
        Point  target = endPt;
        double kappa;

        double dxEnd = endPt.x - robot.x;
        double dyEnd = endPt.y - robot.y;
        double distToEnd = sqrt(dxEnd*dxEnd + dyEnd*dyEnd);

        if (distToEnd < endToleranceMm) break;

        updateRobotSegment(robot);

        // Stop once the robot crosses the line through the end point
        // perpendicular to the last segment. The lookahead is extended past
        // the end, so without this the robot would keep driving.
        if (robotSegment == pathLength - 2 &&
            projectOnSegment(robot, robotSegment) >= 1.0) break;

        double distLeft;
        if (findLookaheadPoint(robot, lookaheadMm, look)) {
            kappa    = curvatureTo(robot, look);
            distLeft = distanceToPathEnd(robot);
            target   = look;
        } else {
            distLeft = distToEnd;
            kappa    = curvatureTo(robot, endPt);
        }

        // Never plan to arrive anywhere farther than the end point itself.
        distLeft = fmin(distLeft, distToEnd);
        // Brake early for upcoming corners (expressed as an equivalent distLeft
        // so targetVelocity's decel cap enforces it).
        double cornerCap = cornerSpeedCap(robot);
        distLeft = fmin(distLeft, cornerCap * cornerCap / (2.0 * maxAccelMmS2));

        // Stall guard: if we are commanding motion but not moving, something
        // is blocking us (wall, pinned robot). Stop rather than burn motors.
        double movedX = robot.x - stallRef.x;
        double movedY = robot.y - stallRef.y;
        if (sqrt(movedX*movedX + movedY*movedY) > stallMinMoveMm) {
            stallRef = robot;
            stallTimerMs = 0.0;
        } else {
            stallTimerMs += controlLoopMs;
            if (stallTimerMs >= stallCheckMs) {
                if (debugTelemetry) {
                    printf("stall detected at (%.1f, %.1f) distLeft:%.1f\n",
                           robot.x, robot.y, distLeft);
                    fflush(stdout);
                }
                break;
            }
        }

        vel = targetVelocity(kappa, distLeft, vel, dt);
        driveWithCurvature(vel, kappa);

        if (debugTelemetry && (debugCounter++ % debugPrintLoopInterval == 0)) {
            double targetHeading = atan2(target.y - robot.y, target.x - robot.x);
            printf("seg:%d robot:(%.1f, %.1f) hdg:%.1f target:(%.1f, %.1f) tHead:%.1f k:%.4f v:%.0f\n",
                   progress.segment, robot.x, robot.y,
                   robot.theta * 180.0 / M_PI,
                   target.x, target.y,
                   targetHeading * 180.0 / M_PI,
                   kappa, vel);
            fflush(stdout);
        }

        this_thread::sleep_for((int)controlLoopMs);
    }

    stopDrive();
}