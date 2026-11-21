#include "wallReset.h"
#include "pursuit.h"
#include "motorDefs.h"
#include <math.h>
#include <stdio.h>

using namespace vex;

namespace {

struct WallSensor {
    vex::distance* dev;
    double fwdMm;
    double leftMm;
    double mountRad;   // beam direction relative to robot heading (0 = front, +pi/2 = left)
};

WallSensor sensors[] = {
    { &leftWallDist,  leftDistFwdMm,  leftDistLeftMm,  M_PI / 2.0 },
    { &frontWallDist, frontDistFwdMm, frontDistLeftMm, 0.0 },
};
constexpr int sensorCount = 2;

struct Walls { double xMin, xMax, yMin, yMax; };

// Square field centered on the origin: identical walls with or without path mirroring.
Walls currentWalls() {
    return { wallXMinMm, wallXMaxMm, wallYMinMm, wallYMaxMm };
}

const char* failReason = "";

bool readSensor(const WallSensor& s, double& d) {
    if (!s.dev->installed()) { failReason = "not installed"; return false; }
    d = s.dev->objectDistance(distanceUnits::mm);
    if (d < wallResetMinRangeMm || d > wallResetMaxRangeMm) { failReason = "out of range"; return false; }
    return true;
}

// Offset (field frame) of the sensor from the pose point.
void sensorOffset(const WallSensor& s, double theta, double& ox, double& oy) {
    double c = cos(theta), sn = sin(theta);
    ox = s.fwdMm * c - s.leftMm * sn;
    oy = s.fwdMm * sn + s.leftMm * c;
}

// Position correction implied by one reading. axis 0 = x, 1 = y; corr is added to the pose.
bool wallFix(const WallSensor& s, const Pose& p, double d, int& axis, double& corr) {
    double ox, oy;
    sensorOffset(s, p.theta, ox, oy);
    double sx = p.x + ox, sy = p.y + oy;

    double a  = p.theta + s.mountRad;
    double ux = cos(a), uy = sin(a);
    Walls w = currentWalls();

    // Nearest wall the beam would hit from where odom thinks the robot is.
    double bestT = 1e18;
    int    wall  = -1;   // 0 xMin, 1 xMax, 2 yMin, 3 yMax
    double t;
    if (ux < -1e-6 && (t = (w.xMin - sx) / ux) > 0.0 && t < bestT) { bestT = t; wall = 0; }
    if (ux >  1e-6 && (t = (w.xMax - sx) / ux) > 0.0 && t < bestT) { bestT = t; wall = 1; }
    if (uy < -1e-6 && (t = (w.yMin - sy) / uy) > 0.0 && t < bestT) { bestT = t; wall = 2; }
    if (uy >  1e-6 && (t = (w.yMax - sy) / uy) > 0.0 && t < bestT) { bestT = t; wall = 3; }
    if (wall < 0) { failReason = "beam hits no wall"; return false; }

    bool vertical = wall < 2;   // wall at constant x
    double incidence = vertical ? fabs(uy) : fabs(ux);   // sin of angle off the wall normal
    if (incidence > sin(wallResetMaxIncidenceRad)) { failReason = "bad incidence angle"; return false; }

    if (vertical) {
        double wallX = (wall == 0) ? w.xMin : w.xMax;
        corr = (wallX - d * ux - ox) - p.x;
        axis = 0;
    } else {
        double wallY = (wall == 2) ? w.yMin : w.yMax;
        corr = (wallY - d * uy - oy) - p.y;
        axis = 1;
    }
    if (fabs(corr) > wallResetMaxCorrectionMm) { failReason = "correction too large"; return false; }
    return true;
}

double median(double* v, int n) {
    for (int i = 1; i < n; i++) {
        double key = v[i];
        int j = i - 1;
        while (j >= 0 && v[j] > key) { v[j + 1] = v[j]; j--; }
        v[j + 1] = key;
    }
    return (n & 1) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

}  // namespace

bool resetPositionFromWalls(bool useLeft, bool useFront, int samples) {
    if (samples < 1) samples = 1;
    if (samples > maxWallSamples) samples = maxWallSamples;
    bool use[sensorCount] = { useLeft, useFront };

    double axisSum[2] = {0.0, 0.0};
    int    axisCnt[2] = {0, 0};

    for (int i = 0; i < sensorCount; i++) {
        if (!use[i] || !sensors[i].dev->installed()) continue;

        double vals[maxWallSamples];
        int    n = 0;
        int    sensorAxis = -1;

        for (int k = 0; k < samples; k++) {
            Pose p = getPose();
            double d;
            int axis;
            double corr;
            if (readSensor(sensors[i], d) && wallFix(sensors[i], p, d, axis, corr)) {
                if (sensorAxis < 0) sensorAxis = axis;
                if (axis == sensorAxis) vals[n++] = corr;
            } else if (k == 0) {
                printf("wall reset: sensor %d rejected (%s) d:%.0f pose:(%.0f,%.0f,%.1fdeg)\n",
                       i, failReason, d, p.x, p.y, p.theta * 180.0 / M_PI);
                fflush(stdout);
            }
            this_thread::sleep_for(wallSampleMs);
        }

        // Need at least half of the samples to agree it sees a wall.
        if (n == 0 || n * 2 < samples) continue;
        axisSum[sensorAxis] += median(vals, n);
        axisCnt[sensorAxis]++;
    }

    double dx = axisCnt[0] ? axisSum[0] / axisCnt[0] : 0.0;
    double dy = axisCnt[1] ? axisSum[1] / axisCnt[1] : 0.0;
    if (!axisCnt[0] && !axisCnt[1]) {
        printf("wall reset: no valid wall reading\n");
        fflush(stdout);
        return false;
    }

    Pose before = getPose();
    adjustPose(dx, dy);
    Pose after = getPose();
    printf("wall reset: from (%.1f, %.1f) to (%.1f, %.1f)  d:(%.1f, %.1f)\n",
           before.x, before.y, after.x, after.y, dx, dy);
    fflush(stdout);
    return true;
}

void wallCalibrationMode() {
    const char* names[sensorCount] = { "left ", "front" };
    while (true) {
        Pose p = getPose();
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("pose x:%.0f y:%.0f h:%.1f", p.x, p.y, p.theta * 180.0 / M_PI);

        for (int i = 0; i < sensorCount; i++) {
            Brain.Screen.setCursor(2 + i, 1);
            if (!sensors[i].dev->installed()) {
                Brain.Screen.print("%s: not installed", names[i]);
                continue;
            }
            double d = sensors[i].dev->objectDistance(distanceUnits::mm);
            double ox, oy;
            sensorOffset(sensors[i], p.theta, ox, oy);
            double a = p.theta + sensors[i].mountRad;
            double hx = p.x + ox + d * cos(a);
            double hy = p.y + oy + d * sin(a);
            Brain.Screen.print("%s: %.0f  hit x:%.0f y:%.0f", names[i], d, hx, hy);
        }
        this_thread::sleep_for(100);
    }
}
