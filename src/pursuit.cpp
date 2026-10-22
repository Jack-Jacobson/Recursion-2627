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
    return path1[targetIndex];
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



// Field heading (rad, CCW +).
static double inertialThetaRad() {
    return startHeadingRad
         - inertialSensor.rotation(rotationUnits::deg) * gyroScale * M_PI / 180.0;
}

void initOdom() {
    inertialSensor.calibrate();
    Brain.Screen.print("Calibrating Inertial Sensor...");
    while (inertialSensor.isCalibrating()) {
        this_thread::sleep_for(10);
    }

    // Zero after calibration so bumps during it aren't counted as travel.
    odomPod.resetPosition();
    this_thread::sleep_for(50);
    prevOdomDeg = odomPod.position(rotationUnits::deg);

    poseMutex.lock();
    pose.x     = path1[0].x;
    pose.y     = path1[0].y;
    pose.theta = inertialThetaRad();
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

// Call instead of followPath(), move the robot by hand and read the brain screen.
void odomCalibrationMode() {
    while (true) {
        double mm  = odomPod.position(rotationUnits::deg) / 360.0 * wheelCircumferenceMm;
        double deg = inertialSensor.rotation(rotationUnits::deg);
        Pose p = getPose();
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("wheel mm: %.1f", mm);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("imu deg:  %.2f", deg);
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("pose x:%.0f y:%.0f h:%.1f", p.x, p.y, p.theta * 180.0 / M_PI);
        this_thread::sleep_for(100);
    }
}

// Both sides scale together past the limit so curvature is preserved.
void setDriveVolts(double leftVolts, double rightVolts) {
    double peak = fmax(fabs(leftVolts), fabs(rightVolts));
    if (peak > maxDriveVolts) {
        double scale = maxDriveVolts / peak;
        leftVolts  *= scale;
        rightVolts *= scale;
    }

    frontLeftDrive.spin(directionType::fwd,  leftVolts,  voltageUnits::volt);
    midLeftDrive.spin(directionType::fwd,    leftVolts,  voltageUnits::volt);
    backLeftDrive.spin(directionType::fwd,   leftVolts,  voltageUnits::volt);

    frontRightDrive.spin(directionType::fwd, rightVolts, voltageUnits::volt);
    midRightDrive.spin(directionType::fwd,   rightVolts, voltageUnits::volt);
    backRightDrive.spin(directionType::fwd,  rightVolts, voltageUnits::volt);
}

void setDrive(double left, double right) {
    double peak = fmax(fabs(left), fabs(right));
    if (peak > maxPowerPct) {
        double scale = maxPowerPct / peak;
        left  *= scale;
        right *= scale;
    }
    setDriveVolts(left  * maxDriveVolts / 100.0,
                  right * maxDriveVolts / 100.0);
}

void driveToTarget(const Pose& robot, const Point& target) {
    double error = headingErrorToTarget(target, robot);

    double turnPower = error * turnKp;

    setDrive(forwardPowerPct - turnPower, forwardPowerPct + turnPower);
}

void setDriveBrake(brakeType mode) {
    frontLeftDrive.stop(mode);
    midLeftDrive.stop(mode);
    backLeftDrive.stop(mode);
    frontRightDrive.stop(mode);
    midRightDrive.stop(mode);
    backRightDrive.stop(mode);
}

void stopDrive() {
    setDriveBrake(brakeType::brake);
}

double lookaheadFor(double velMmS) {
    return fmin(fmax(velMmS * lookaheadTimeS, minLookaheadMm), lookaheadMm);
}

static const Point* activePath = path1;
static int          activeLen  = path1Length;
static PathOpts     activeOpts;

static PathProgress progress;
static Point lastLook;
static bool  haveLastLook = false;

// Unclamped fraction along segment i of the robot's projection.
static double projectOnSegment(const Pose& robot, int i) {
    double dx = activePath[i + 1].x - activePath[i].x;
    double dy = activePath[i + 1].y - activePath[i].y;
    double lenSq = dx*dx + dy*dy;
    if (lenSq < 1e-9) return 1.0;
    return ((robot.x - activePath[i].x) * dx + (robot.y - activePath[i].y) * dy) / lenSq;
}

static double segmentLength(int i) {
    double dx = activePath[i + 1].x - activePath[i].x;
    double dy = activePath[i + 1].y - activePath[i].y;
    return sqrt(dx*dx + dy*dy);
}

// Picks the furthest-along intersection of the lookahead circle with the path
bool findLookaheadPoint(const Pose& robot, double radius, Point& out) {
    bool found = false;
    PathProgress best = progress;
    Point bestPt = {0, 0};

    for (int i = progress.segment; i < activeLen - 1; i++) {
        Point E = activePath[i]; //start of segment
        Point L = activePath[i + 1]; //end of segment

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

        // The last segment is extended past the end point for STEERING only
        double tMax = (i == activeLen - 2) ? 1e9 : 1.0;
        double t = -1.0;
        if      (t2 >= 0.0 && t2 <= tMax) t = t2;
        else if (t1 >= 0.0 && t1 <= tMax) t = t1;
        if (t < 0.0) continue;

        // Never move backwards along the segment
        if (i == progress.segment && t < progress.t) t = progress.t;

        // Keep looping when later segments are further along the path.
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

// Measured wheel surface speed (mm/s) of one side.
static double sideVelocityMmS(motor& a, motor& b, motor& c) {
    double rpm = (a.velocity(velocityUnits::rpm) +
                  b.velocity(velocityUnits::rpm) +
                  c.velocity(velocityUnits::rpm)) / 3.0;
    return rpm * wheelPerMotor / 60.0 * M_PI * driveWheelMm;
}

static double wheelVolts(double targetVel, double targetAccel, double measuredVel) {
    double volts = driveKv * targetVel
                 + driveKa * targetAccel
                 + driveKp * (targetVel - measuredVel);
    if (fabs(targetVel) > 1e-3) volts += copysign(driveKs, targetVel);
    return volts;
}

static double prevLeftTargetMmS  = 0.0;
static double prevRightTargetMmS = 0.0;

static double headingHist[yawRateWindow + 1];
static int    headingHistCount = 0;
static int    headingHistIdx   = 0;
static double yawErrIntegral   = 0.0;

static void resetDriveFeedforward() {
    prevLeftTargetMmS  = 0.0;
    prevRightTargetMmS = 0.0;
    headingHistCount   = 0;
    headingHistIdx     = 0;
    yawErrIntegral     = 0.0;
}

// Yaw rate (rad/s, CCW +) over the last yawRateWindow loops; false until filled.
static bool measuredYawRate(double& out) {
    double theta = getPose().theta;
    headingHist[headingHistIdx] = theta;
    int oldest = (headingHistIdx + 1) % (yawRateWindow + 1);
    headingHistIdx = oldest;
    if (headingHistCount < yawRateWindow + 1) {
        headingHistCount++;
        if (headingHistCount < yawRateWindow + 1) return false;
    }
    double dTheta = normalizeAngle(theta - headingHist[oldest]);
    out = dTheta / (yawRateWindow * controlLoopMs / 1000.0);
    return true;
}

void driveWithCurvature(double v, double kappa) {
    driveWithYawRate(v, v * kappa);
}

// v: mm/s (negative = reverse). omega: rad/s, CCW +.
void driveWithYawRate(double v, double omega) {
    double offset = omega * curvatureGain * (trackWidthMm / 2.0);
    double leftTarget  = v - offset;
    double rightTarget = v + offset;

    const double dt = controlLoopMs / 1000.0;
    double leftAccel  = (leftTarget  - prevLeftTargetMmS)  / dt;
    double rightAccel = (rightTarget - prevRightTargetMmS) / dt;
    leftAccel  = fmin(fmax(leftAccel,  -maxFeedforwardAccelMmS2), maxFeedforwardAccelMmS2);
    rightAccel = fmin(fmax(rightAccel, -maxFeedforwardAccelMmS2), maxFeedforwardAccelMmS2);
    prevLeftTargetMmS  = leftTarget;
    prevRightTargetMmS = rightTarget;

    double leftMeas  = sideVelocityMmS(frontLeftDrive,  midLeftDrive,  backLeftDrive);
    double rightMeas = sideVelocityMmS(frontRightDrive, midRightDrive, backRightDrive);

    double leftV  = wheelVolts(leftTarget,  leftAccel,  leftMeas);
    double rightV = wheelVolts(rightTarget, rightAccel, rightMeas);

    // Gyro yaw-rate PI: positive error = needs more CCW turn.
    double yawMeas;
    if (measuredYawRate(yawMeas)) {
        double yawErr = omega - yawMeas;
        yawErrIntegral = yawErrIntegral * fmax(0.0, 1.0 - dt / yawRateILeakS) + yawErr * dt;
        if (yawRateKi > 0.0) {
            double iMax = yawRateMaxVolts / yawRateKi;
            yawErrIntegral = fmin(fmax(yawErrIntegral, -iMax), iMax);
        }
        double corr = yawRateKp * yawErr + yawRateKi * yawErrIntegral;
        corr = fmin(fmax(corr, -yawRateMaxVolts), yawRateMaxVolts);
        leftV  -= corr;
        rightV += corr;
    }

    setDriveVolts(leftV, rightV);
}

// flags

static PathFlag flags[maxFlags];
static double   flagPathMm[maxFlags];   // trigger distance along the path
static double   flagStopMm[maxFlags];   // stop distance along the path
static Point    flagSnap[maxFlags];     // flag point snapped onto the path
static bool     flagFired[maxFlags];
static bool     flagStopDone[maxFlags];
static int      flagCount = 0;

static void pushFlag(const PathFlag& f) {
    if (flagCount >= maxFlags || f.action == nullptr) {
        if (debugTelemetry) {
            printf("addFlag ignored (full or null action)\n");
            fflush(stdout);
        }
        return;
    }
    flags[flagCount++] = f;
}

void addFlag(const Point& at, FlagAction action, double leadMm) {
    pushFlag({at, action, leadMm, false, 0});
}

void addStopFlag(const Point& at, FlagAction action, int waitMs, double leadMm) {
    pushFlag({at, action, leadMm, true, waitMs});
}

void clearFlags() {
    flagCount = 0;
}

// Distance along the path of the path point closest to p.
static double pathDistanceOf(const Point& p, Point* snapped = nullptr) {
    double bestDistSq = 1e18;
    double bestS = 0.0;
    double s = 0.0;
    for (int i = 0; i < activeLen - 1; i++) {
        double len = segmentLength(i);
        double t = 0.0;
        if (len > 1e-9) {
            t = ((p.x - activePath[i].x) * (activePath[i + 1].x - activePath[i].x) +
                 (p.y - activePath[i].y) * (activePath[i + 1].y - activePath[i].y)) / (len * len);
            t = fmin(fmax(t, 0.0), 1.0);
        }
        double px = activePath[i].x + t * (activePath[i + 1].x - activePath[i].x) - p.x;
        double py = activePath[i].y + t * (activePath[i + 1].y - activePath[i].y) - p.y;
        double dSq = px*px + py*py;
        if (dSq < bestDistSq) {
            bestDistSq = dSq;
            bestS = s + t * len;
            if (snapped) *snapped = {px + p.x, py + p.y};
        }
        s += len;
    }
    return bestS;
}

static double robotPathDistance(const Pose& robot, int seg) {
    double s = 0.0;
    for (int i = 0; i < seg; i++) s += segmentLength(i);
    double t = fmin(fmax(projectOnSegment(robot, seg), 0.0), 1.0);
    return s + t * segmentLength(seg);
}

static void resetFlags() {
    for (int i = 0; i < flagCount; i++) {
        flagStopMm[i]   = pathDistanceOf(flags[i].at, &flagSnap[i]);
        flagPathMm[i]   = flagStopMm[i] - flags[i].leadMm;
        flagFired[i]    = false;
        flagStopDone[i] = !flags[i].stop;
    }
}

static uint32_t flagFiredAtMs[maxFlags];

static void fireFlag(int i, const Pose& robot) {
    flagFired[i] = true;
    flagFiredAtMs[i] = timer::system();
    if (debugTelemetry) {
        printf("flag %d fired at r:(%.0f,%.0f) for (%.0f,%.0f)\n",
               i, robot.x, robot.y, flags[i].at.x, flags[i].at.y);
        fflush(stdout);
    }
    flags[i].action();
}

// Next unserviced stop flag (closest along the path), or -1.
static int nextStopFlag() {
    int best = -1;
    for (int i = 0; i < flagCount; i++) {
        if (flagStopDone[i]) continue;
        if (best < 0 || flagStopMm[i] < flagStopMm[best]) best = i;
    }
    return best;
}

static void serviceStopFlag(int i) {
    setDriveBrake(brakeType::hold);
    if (!flagFired[i]) fireFlag(i, getPose());

    while ((int)(timer::system() - flagFiredAtMs[i]) < flags[i].waitMs) {
        this_thread::sleep_for(5);
    }
    flagStopDone[i] = true;

    if (debugTelemetry) {
        printf("stop flag %d done after %lu ms\n",
               i, (unsigned long)(timer::system() - flagFiredAtMs[i]));
        fflush(stdout);
    }
}

static void checkFlags(const Pose& robot, double& progressMm, int seg) {
    progressMm = fmax(progressMm, robotPathDistance(robot, seg));
    for (int i = 0; i < flagCount; i++) {
        if (!flagFired[i] && progressMm >= flagPathMm[i]) fireFlag(i, robot);
    }
}

static int robotSegment = 0;   // segment the robot is physically on

double distanceToPathEnd(const Pose& robot) {
    int seg = robotSegment;
    double total;
    if (seg == activeLen - 2) {
        // Measured along the last segment so decel ends at the end line.
        total = segmentLength(seg) * (1.0 - projectOnSegment(robot, seg));
    } else {
        Point B = activePath[seg + 1];
        double dx = B.x - robot.x;
        double dy = B.y - robot.y;
        total = sqrt(dx*dx + dy*dy);
    }

    for (int i = seg + 1; i < activeLen - 1; i++) {
        total += segmentLength(i);
    }
    return total;
}

static double distToSegment(const Pose& robot, int i) {
    double t = fmin(fmax(projectOnSegment(robot, i), 0.0), 1.0);
    double px = activePath[i].x + t * (activePath[i + 1].x - activePath[i].x) - robot.x;
    double py = activePath[i].y + t * (activePath[i + 1].y - activePath[i].y) - robot.y;
    return sqrt(px*px + py*py);
}

// Advances when past the segment end OR closer to the next one (pursuit cuts corners).
static void updateRobotSegment(const Pose& robot) {
    while (robotSegment < activeLen - 2) {
        int n = robotSegment + 1;
        bool past   = projectOnSegment(robot, robotSegment) >= 1.0;
        bool closer = projectOnSegment(robot, n) > 0.0 &&
                      distToSegment(robot, n) < distToSegment(robot, robotSegment);
        if (!past && !closer) break;
        robotSegment = n;
    }
}

// Speed cap so the robot brakes early enough for the arc pursuit cuts through upcoming corners.
double cornerSpeedCap(const Pose& robot) {
    const double vMax = activeOpts.maxVel_;
    const double aMax = activeOpts.maxAccel_;
    double cap = vMax;

    double t = fmin(fmax(projectOnSegment(robot, robotSegment), 0.0), 1.0);
    double distToCorner = segmentLength(robotSegment) * (1.0 - t);

    for (int j = robotSegment + 1; j < activeLen - 1; j++) {
        double ax = activePath[j].x - activePath[j - 1].x, ay = activePath[j].y - activePath[j - 1].y;
        double bx = activePath[j + 1].x - activePath[j].x, by = activePath[j + 1].y - activePath[j].y;
        double turn = fabs(normalizeAngle(atan2(by, bx) - atan2(ay, ax)));
        turn = fmin(turn, cornerMaxAngleRad);

        // Arc depends on lookahead, which depends on speed: iterate to converge.
        double tanHalf = tan(turn / 2.0);
        double vCorner = vMax;
        if (tanHalf > 1e-6) {
            for (int iter = 0; iter < 3; iter++) {
                double kCorner = 2.0 * tanHalf / lookaheadFor(vCorner);
                double vc = sqrt(maxLatAccelMmS2 / kCorner);
                vc = fmin(vc, maxWheelVelMmS / (1.0 + kCorner * trackWidthMm / 2.0));
                vCorner = fmin(vc, vMax);
            }
        }
        vCorner = fmax(vCorner, fmin(minVelocityMmS, vMax));

        cap = fmin(cap, sqrt(vCorner*vCorner + 2.0 * aMax * distToCorner));
        distToCorner += segmentLength(j);
    }
    return cap;
}

double targetVelocity(double kappa, double distLeft, double prevVel, double dt) {
    const double vMax = activeOpts.maxVel_;
    const double aMax = activeOpts.maxAccel_;
    double v = vMax;

    double curveCap = vMax;
    if (fabs(kappa) > 1e-6)
        curveCap = sqrt(maxLatAccelMmS2 / fabs(kappa));

    // Keep the outer wheel within motor limits, or setDriveVolts scaling changes the curvature.
    double outerRatio = 1.0 + fabs(kappa) * (trackWidthMm / 2.0);
    curveCap = fmin(curveCap, maxWheelVelMmS / outerRatio);

    v = fmin(v, curveCap);

    double decelCap = sqrt(2.0 * aMax * fmax(distLeft, 0.0));
    v = fmin(v, decelCap);

    v = fmin(v, prevVel + aMax * dt);

    // Min-speed floor for the initial ramp; never overrides corner/stop caps.
    if (v > 0.0 && v < minVelocityMmS) {
        v = fmin(fmin(minVelocityMmS, vMax), fmin(curveCap, decelCap));
    }

    return fmax(v, 0.0);
}

// Motion-profiled turn: commands a decelerating turn rate closed-loop on the gyro.
// errorOf(pose) returns the remaining heading error (rad, CCW +).
template <typename ErrorFn>
static void profiledTurn(ErrorFn errorOf, double toleranceRad) {
    int settled = 0;
    int elapsedMs = 0;
    const double dt = controlLoopMs / 1000.0;
    const double wheelPerRad = turnScrubFactor * trackWidthMm / 2.0;   // rad/s -> wheel mm/s

    double cmdOmega = 0.0;
    double rateI    = 0.0;
    double measRate = 0.0;
    double prevTheta = getPose().theta;
    double prevError = errorOf(getPose());

    while (elapsedMs < maxTurnMs) {
        Pose robot = getPose();
        double error = errorOf(robot);

        double rawRate = normalizeAngle(robot.theta - prevTheta) / dt;
        prevTheta = robot.theta;
        measRate += 0.4 * (rawRate - measRate);

        // Overshot: restart the profile in the other direction.
        if (prevError * error < 0.0) {
            cmdOmega = 0.0;
            rateI    = 0.0;
        }
        prevError = error;

        bool inTol = fabs(error) < toleranceRad;
        if (inTol) {
            if (++settled >= startTurnSettleLoops) break;
        } else {
            settled = 0;
        }

        double prevCmd = cmdOmega;
        if (inTol) {
            cmdOmega = 0.0;
        } else {
            // Fastest rate that can still stop at the goal, accel-limited, with a min floor.
            double remaining = fmax(fabs(error) - copysign(1.0, error) * measRate * turnLagCompS, 0.0);
            double mag = fmin(turnMaxOmegaRadS, sqrt(2.0 * turnDecelRadS2 * remaining));
            mag = fmin(mag, fabs(prevCmd) + turnAccelRadS2 * dt);
            mag = fmax(mag, turnMinOmegaRadS);
            cmdOmega = copysign(mag, error);
        }
        double cmdAccel = fmin(fmax((cmdOmega - prevCmd) / dt, -turnAccelRadS2), turnAccelRadS2);

        double volts = 0.0;
        if (!inTol) {
            double rateErr = cmdOmega - measRate;
            rateI += rateErr * dt;
            double iMax = turnRateIMaxVolts / turnRateKi;
            rateI = fmin(fmax(rateI, -iMax), iMax);

            volts = copysign(turnKs, cmdOmega)
                  + driveKv * cmdOmega * wheelPerRad
                  + driveKa * cmdAccel * wheelPerRad
                  + turnRateKp * rateErr
                  + turnRateKi * rateI;
        }

        if (debugTelemetry && (elapsedMs % 50 == 0)) {
            printf("turn hdg:%.1f err:%.1f w:%.0f meas:%.0f V:%.1f\n",
                   robot.theta * 180.0 / M_PI, error * 180.0 / M_PI,
                   cmdOmega * 180.0 / M_PI, measRate * 180.0 / M_PI, volts);
            fflush(stdout);
        }

        if (inTol) setDriveBrake(brakeType::hold);
        else       setDriveVolts(-volts, volts);

        this_thread::sleep_for((int)controlLoopMs);
        elapsedMs += (int)controlLoopMs;
    }
    stopDrive();

    if (debugTelemetry && elapsedMs >= maxTurnMs) {
        printf("turn timeout, err:%.1f\n", errorOf(getPose()) * 180.0 / M_PI);
        fflush(stdout);
    }
    this_thread::sleep_for(50);
}

void turnToFace(const Point& target, double toleranceRad) {
    profiledTurn([&](const Pose& p) { return headingErrorToTarget(target, p); },
                 toleranceRad);
}

void turnToHeading(double headingDeg, double toleranceRad) {
    const double goalRad = normalizeAngle(headingDeg * M_PI / 180.0);
    profiledTurn([goalRad](const Pose& p) { return normalizeAngle(goalRad - p.theta); },
                 toleranceRad);
}

// run timer

static uint32_t runTimerStartMs   = 0;
static uint32_t runTimerExcludeMs = 0;   // debug-only END settle pauses

void startRunTimer() {
    if (!timeRuns) return;
    runTimerStartMs   = timer::system();
    runTimerExcludeMs = 0;
    printf("RUN TIMER started\n");
    fflush(stdout);
}

double endRunTimer() {
    if (!timeRuns) return 0.0;
    uint32_t total = timer::system() - runTimerStartMs;
    double secs = (total - runTimerExcludeMs) / 1000.0;
    if (runTimerExcludeMs > 0) {
        printf("RUN TIME: %.2f s (%.2f s wall, %.2f s of debug settle excluded)\n",
               secs, total / 1000.0, runTimerExcludeMs / 1000.0);
    } else {
        printf("RUN TIME: %.2f s\n", secs);
    }
    fflush(stdout);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Run time: %.2f s", secs);
    return secs;
}

void driveDistance(double distMm, double maxVel) {
    resetDriveFeedforward();

    const Pose   start   = getPose();
    const double holdHdg = start.theta;
    const double dir     = (distMm >= 0.0) ? 1.0 : -1.0;
    const double goal    = fabs(distMm);
    const double cosH    = cos(holdHdg), sinH = sin(holdHdg);
    const double dt      = controlLoopMs / 1000.0;
    maxVel = fmin(fabs(maxVel), maxVelMmS);

    double vel = 0.0;
    int elapsedMs = 0;
    Pose stallRef = start;
    double stallTimerMs = 0.0;

    while (elapsedMs < driveDistTimeoutMs) {
        Pose robot = getPose();

        // Measured along the start heading so sideways drift isn't progress.
        double travelled = dir * ((robot.x - start.x) * cosH + (robot.y - start.y) * sinH);
        double remaining = goal - travelled;
        if (remaining < driveDistToleranceMm) break;

        double moveX = robot.x - stallRef.x, moveY = robot.y - stallRef.y;
        if (sqrt(moveX*moveX + moveY*moveY) > stallMinMoveMm) {
            stallRef = robot;
            stallTimerMs = 0.0;
        } else if ((stallTimerMs += controlLoopMs) >= stallCheckMs) {
            if (debugTelemetry) {
                printf("driveDistance stall, remaining:%.1f\n", remaining);
                fflush(stdout);
            }
            break;
        }

        double decelCap = sqrt(2.0 * maxAccelMmS2 * remaining);
        vel = fmin(fmin(maxVel, decelCap), vel + maxAccelMmS2 * dt);
        if (vel < minVelocityMmS) vel = fmin(minVelocityMmS, decelCap);

        double hdgErr = normalizeAngle(holdHdg - robot.theta);
        double omega  = fmin(fmax(hdgErr * driveDistHeadingKp, -maxHoldYawRate), maxHoldYawRate);

        driveWithYawRate(dir * vel, omega);

        if (debugTelemetry && (elapsedMs % 100 == 0)) {
            printf("drive rem:%.0f v:%.0f hdgErr:%.2f\n",
                   remaining, dir * vel, hdgErr * 180.0 / M_PI);
            fflush(stdout);
        }

        this_thread::sleep_for((int)controlLoopMs);
        elapsedMs += (int)controlLoopMs;
    }
    stopDrive();
}

void followPath(int pathNumber, const PathOpts& opts) {
    if (pathNumber < 1 || pathNumber > pathCount) {
        if (debugTelemetry) {
            printf("followPath(%d): no such path (have 1-%d)\n", pathNumber, pathCount);
            fflush(stdout);
        }
        return;
    }
    const PathRef& p = paths[pathNumber - 1];
    followPath(p.pts, p.len, opts);
}

// Heading pursuit steers with: the back of the robot when reversing.
static Pose steerPose(const Pose& robot) {
    Pose s = robot;
    if (activeOpts.reverse_) s.theta = normalizeAngle(robot.theta + M_PI);
    return s;
}

void followPath(const Point* pts, int len, const PathOpts& opts) {
    if (pts == nullptr || len < 2) return;
    activePath = pts;
    activeLen  = len;
    activeOpts = opts;
    activeOpts.maxVel_   = fmin(fmax(opts.maxVel_, 1.0), maxVelMmS);
    activeOpts.maxAccel_ = fmax(opts.maxAccel_, 1.0);

    progress     = PathProgress();
    haveLastLook = false;
    robotSegment = 0;
    resetDriveFeedforward();
    resetFlags();
    double flagProgressMm = 0.0;
    bool   stalled = false;

    // Flags at the very start fire before the robot moves.
    checkFlags(getPose(), flagProgressMm, 0);
    for (int s = nextStopFlag(); s >= 0 && flagStopMm[s] - flagProgressMm < stopFlagToleranceMm;
         s = nextStopFlag()) {
        serviceStopFlag(s);
    }

    // Start turn aims at the first lookahead point (not path[1]) so it agrees with pursuit.
    uint32_t pathStartMs = timer::system();
    if (activeOpts.startTurn_) {
        Point aim;
        if (!findLookaheadPoint(getPose(), lookaheadFor(0.0), aim)) aim = activePath[activeLen - 1];
        progress     = PathProgress();
        haveLastLook = false;
        auto aimErr = [&](const Pose& p) { return headingErrorToTarget(aim, steerPose(p)); };
        if (fabs(aimErr(getPose())) > startTurnSkipRad) {
            profiledTurn(aimErr, pathStartTurnToleranceRad);
        }
    }
    uint32_t turnDoneMs = timer::system();

    double vel = 0.0;
    const double dt = controlLoopMs / 1000.0;
    int debugCounter = 0;

    const Point endPt = activePath[activeLen - 1];

    Pose stallRef = getPose();
    double stallTimerMs = 0.0;

    while (true) {
        Pose   robot = getPose();
        Pose   steer = steerPose(robot);
        Point  look;
        Point  target = endPt;
        double kappa;

        double dxEnd = endPt.x - robot.x;
        double dyEnd = endPt.y - robot.y;
        double distToEnd = sqrt(dxEnd*dxEnd + dyEnd*dyEnd);

        if (distToEnd < activeOpts.endTol_) break;

        updateRobotSegment(robot);
        checkFlags(robot, flagProgressMm, robotSegment);

        int    stopIdx  = nextStopFlag();
        double distStop = 1e9;
        if (stopIdx >= 0) {
            double sx = flagSnap[stopIdx].x - robot.x;
            double sy = flagSnap[stopIdx].y - robot.y;
            distStop = flagStopMm[stopIdx] - flagProgressMm;
            // Also accept being physically close (progress lags when cutting corners).
            if (distStop < stopFlagToleranceMm ||
                (distStop < lookaheadMm && sqrt(sx*sx + sy*sy) < stopFlagToleranceMm)) {
                serviceStopFlag(stopIdx);
                vel = 0.0;
                resetDriveFeedforward();
                stallRef = getPose();
                stallTimerMs = 0.0;
                continue;
            }
        }

        // Stop once the robot crosses the line through the end point.
        if (robotSegment == activeLen - 2 &&
            projectOnSegment(robot, robotSegment) >= 1.0) break;

        double lookahead = lookaheadFor(vel);

        double distLeft;
        if (findLookaheadPoint(robot, lookahead, look)) {
            kappa    = curvatureTo(steer, look);
            distLeft = distanceToPathEnd(robot);
            target   = look;
        } else {
            distLeft = distToEnd;
            kappa    = curvatureTo(steer, endPt);
        }

        distLeft = fmin(distLeft, distToEnd);
        distLeft = fmin(distLeft, fmax(distStop, 0.0));
        double cornerCap = cornerSpeedCap(robot);
        distLeft = fmin(distLeft, cornerCap * cornerCap / (2.0 * activeOpts.maxAccel_));

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
                stalled = true;
                break;
            }
        }

        vel = targetVelocity(kappa, distLeft, vel, dt);
        // Reversing: wheels go backward but the yaw rate keeps its sign (v * kappa).
        if (activeOpts.reverse_) driveWithYawRate(-vel, vel * kappa);
        else                     driveWithCurvature(vel, kappa);

        if (debugTelemetry && (debugCounter++ % debugPrintLoopInterval == 0)) {
            double targetHeading = atan2(target.y - robot.y, target.x - robot.x);
            printf("seg:%d r:(%.0f,%.0f) h:%.1f t:(%.0f,%.0f) th:%.1f k:%.4f v:%.0f L:%.0f\n",
                   progress.segment, robot.x, robot.y,
                   robot.theta * 180.0 / M_PI,
                   target.x, target.y,
                   targetHeading * 180.0 / M_PI,
                   kappa, activeOpts.reverse_ ? -vel : vel, lookahead);
            fflush(stdout);
        }

        this_thread::sleep_for((int)controlLoopMs);
    }

    stopDrive();

    // Finished inside the end tolerance: fire anything still pending.
    if (!stalled) {
        Pose fin = getPose();
        for (int i = 0; i < flagCount; i++) {
            if (!flagFired[i] && !flags[i].stop) fireFlag(i, fin);
        }
        for (int s = nextStopFlag(); s >= 0; s = nextStopFlag()) serviceStopFlag(s);
        stopDrive();
    }

    clearFlags();

    if (timeRuns) {
        uint32_t doneMs = timer::system();
        printf("PATH TIME: %.2f s (turn %.2f s, drive %.2f s)\n",
               (doneMs - pathStartMs) / 1000.0,
               (turnDoneMs - pathStartMs) / 1000.0,
               (doneMs - turnDoneMs) / 1000.0);
        fflush(stdout);
    }

    if (debugTelemetry) {
        runTimerExcludeMs += 300;
        this_thread::sleep_for(300);   // settle so the final pose is real
        Pose fin = getPose();
        double ex = fin.x - endPt.x, ey = fin.y - endPt.y;
        printf("END r:(%.1f,%.1f) h:%.1f  err x:%.1f y:%.1f dist:%.1f\n",
               fin.x, fin.y, fin.theta * 180.0 / M_PI, ex, ey, sqrt(ex*ex + ey*ey));
        fflush(stdout);
    }

    activeOpts = PathOpts();
}
