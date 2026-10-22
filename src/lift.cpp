#include "lift.h"

using namespace vex;

static volatile double liftTargetMm  = 0.0;
static volatile double liftHeightMm  = 0.0;
static volatile int    liftSettled   = 0;
static double          liftZeroMm    = 0.0;
static bool            liftStarted   = false;

static double medianBuf[liftMedianSize];
static int    medianIdx = 0;

static void spinLift(double volts) {
    lift1.spin(directionType::fwd, volts, voltageUnits::volt);
    lift2.spin(directionType::fwd, volts, voltageUnits::volt);
    lift3.spin(directionType::fwd, volts, voltageUnits::volt);
}

static void holdLift() {
    lift1.stop(brakeType::hold);
    lift2.stop(brakeType::hold);
    lift3.stop(brakeType::hold);
}

// mm or -1 if invalid
static double readSensorMm() {
    if (!liftSensor.isObjectDetected()) return -1.0;
    double mm = liftSensor.objectDistance(distanceUnits::mm);
    if (mm <= 0.0 || mm > liftSensorMaxMm) return -1.0;
    return mm;
}

static double median(const double* v, int n) {
    double s[liftMedianSize];
    for (int i = 0; i < n; i++) s[i] = v[i];
    for (int i = 1; i < n; i++) {
        double k = s[i];
        int j = i - 1;
        while (j >= 0 && s[j] > k) { s[j + 1] = s[j]; j--; }
        s[j + 1] = k;
    }
    return s[n / 2];
}

// bad reading, get most recent good reading
static double rawHeightMm(double lastHeight) {
    double mm = readSensorMm();
    if (mm < 0.0) return lastHeight;
    return (mm - liftZeroMm);
}

static int liftTask() {
    double height = 0.0;
    for (int i = 0; i < liftMedianSize; i++) medianBuf[i] = 0.0;

    double integral   = 0.0;
    double lastHeight = 0.0;
    double velocity   = 0.0;   // mm/s
    bool   holding    = true;
    uint32_t lastMs   = timer::system();

    while (true) {
        uint32_t now = timer::system();
        double dt = (now - lastMs) / 1000.0;
        lastMs = now;

        medianBuf[medianIdx] = rawHeightMm(height);
        medianIdx = (medianIdx + 1) % liftMedianSize;
        double med = median(medianBuf, liftMedianSize);
        height += liftFilterAlpha * (med - height);
        liftHeightMm = height;

        if (dt > 0) velocity += 0.3 * ((height - lastHeight) / dt - velocity);
        lastHeight = height;

        double error = liftTargetMm - height;
        // Predicted error
        double predError = error - velocity * liftLagCompS;

        if (fabs(error) < liftToleranceMm) {
            if (liftSettled < 1000) liftSettled = liftSettled + 1;
        } else {
            liftSettled = 0;
        }

        if (holding && fabs(error) > liftHoldExitMm)  holding = false;
        if (!holding && (fabs(error) < liftHoldEnterMm ||
                         (fabs(error) < liftHoldExitMm && predError * error <= 0.0)))
            holding = true;

        if (holding) {
            integral = 0.0;
            holdLift();
        } else {
            if (fabs(error) < liftIZoneMm) integral += error;
            else                           integral = 0.0;
            integral = fmin(fmax(integral, -liftMaxIntegral), liftMaxIntegral);

            bool down = error < 0.0;
            double kp  = down ? liftKp * liftDownKpScale : liftKp;
            double kd  = down ? liftDownKd : liftKd;

            double volts = liftKg + kp * error + liftKi * integral
                         - kd * velocity / 1000.0;
            volts = fmin(fmax(volts, down ? -liftMaxDownVolts : -liftMaxVolts), liftMaxVolts);
            spinLift(volts);
        }

        this_thread::sleep_for(liftLoopMs);
    }
    return 0;
}

void startLift() {
    if (liftStarted) return;
    liftStarted = true;

    double sum = 0.0;
    int    n   = 0;
    for (int i = 0; i < 30; i++) {
        double mm = readSensorMm();
        if (mm >= 0.0) { sum += mm; n++; }
        this_thread::sleep_for(10);
    }
    liftZeroMm   = n > 0 ? sum / n : 0.0;
    liftTargetMm = 0.0;
    liftHeightMm = 0.0;

    static vex::thread liftThread(liftTask);
}

void setLiftTarget(double mm) {
    liftTargetMm = fmin(fmax(mm, 0.0), liftMaxHeightMm);
    liftSettled  = 0;
}

double getLiftTarget() { return liftTargetMm; }
double getLiftHeight() { return liftHeightMm; }

bool liftAtTarget() {
    return liftSettled >= liftSettleLoops;
}

void liftTo(double mm, int timeoutMs) {
    setLiftTarget(mm);
    uint32_t start = timer::system();
    this_thread::sleep_for(liftLoopMs * 2);
    while (!liftAtTarget() && (int)(timer::system() - start) < timeoutMs) {
        this_thread::sleep_for(liftLoopMs);
    }
}
