#include "logger.h"
#include "pursuit.h"
#include "lift.h"
#include "drive.h"
#include "motorDefs.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

using namespace vex;

static const int    kSampleMs   = 50;
static const int    kTempEveryMs = 1000;
static const int    kMaxRows    = 2400;     
static const int    kTempCount  = 9;
static const char*  kTempNames[kTempCount] = {
    "drive_fl", "drive_ml", "drive_bl", "drive_fr", "drive_mr", "drive_br",
    "lift1", "lift2", "lift3"
};

struct LogRow {
    uint32_t t_ms;
    float    x, y;      
    int16_t  theta_d10;   
    int16_t  lift_d10;   
    uint8_t  claw;
    uint8_t  period;    
    uint8_t  temps[kTempCount];
};

static LogRow    rows[kMaxRows];
static int       rowCount = 0;
static competition* comp = nullptr;
static volatile bool flushing = false;

static void readTemps(uint8_t* t) {
    motor* m[kTempCount] = { &frontLeftDrive, &midLeftDrive, &backLeftDrive,
                             &frontRightDrive, &midRightDrive, &backRightDrive,
                             &lift1, &lift2, &lift3 };
    for (int i = 0; i < kTempCount; i++) {
        double c = m[i]->installed() ? m[i]->temperature(celsius) : 0.0;
        if (c < 0) c = 0;
        if (c > 255) c = 255;
        t[i] = (uint8_t)(c + 0.5);
    }
}

static int16_t clamp16(double v) {
    if (v > 32767.0) return 32767;
    if (v < -32768.0) return -32768;
    return (int16_t)lrint(v);
}

static void sampleRow(uint32_t t, const uint8_t* temps) {
    if (rowCount >= kMaxRows) return;
    Pose p = getPose();
    double deg = fmod(p.theta * 180.0 / M_PI, 360.0);
    if (deg > 180.0) deg -= 360.0;
    if (deg < -180.0) deg += 360.0;

    LogRow& r = rows[rowCount++];
    r.t_ms      = t;
    r.x         = (float)p.x;
    r.y         = (float)p.y;
    r.theta_d10 = clamp16(deg * 10.0);
    r.lift_d10  = clamp16(getLiftHeight() * 10.0);
    r.claw      = isClawClosed() ? 1 : 0;
    r.period    = (comp && comp->isAutonomous()) ? 1 : 2;
    memcpy(r.temps, temps, kTempCount);
}

static void nextFileName(char* out, size_t n) {
    for (int i = 1; i < 1000; i++) {
        snprintf(out, n, "match_%d.csv", i);
        if (!Brain.SDcard.exists(out)) return;
    }
    snprintf(out, n, "match_999.csv");
}

static char logName[24] = "";
static bool logFileStarted = false;
static int  rowsWritten = 0;
static int  bytesWritten = 0;

static void sdWrite(const char* data, int len) {
    if (len <= 0) return;
    int32_t w;
    if (!logFileStarted) {
        w = Brain.SDcard.savefile(logName, (uint8_t*)data, len);
        logFileStarted = true;
    } else {
        w = Brain.SDcard.appendfile(logName, (uint8_t*)data, len);
    }
    printf("[SD] write %d bytes -> %d\n", len, (int)w);
    if (w > 0) bytesWritten += w;
}

static void loggerBegin() {
    rowsWritten = 0;
    bytesWritten = 0;
    logFileStarted = false;
    if (!Brain.SDcard.isInserted()) {
        printf("[SD] no SD card inserted\n");
        Brain.Screen.printAt(10, 230, false, "LOG: no SD card      ");
        logName[0] = 0;
        return;
    }
    nextFileName(logName, sizeof(logName));
    printf("[SD] file=%s\n", logName);
    static char hdr[256];
    int len = snprintf(hdr, sizeof(hdr), "t_ms,x,y,theta,lift,claw,period");
    for (int i = 0; i < kTempCount; i++) {
        len += snprintf(hdr + len, sizeof(hdr) - len, ",temp_%s", kTempNames[i]);
    }
    len += snprintf(hdr + len, sizeof(hdr) - len, "\n");
    sdWrite(hdr, len);
    printf("[SD] exists after header=%d\n", (int)Brain.SDcard.exists(logName));
}

void loggerFlush() {
    if (flushing || !logName[0]) return;
    flushing = true;
    int end = rowCount;
    static char buf[1280];
    int len = 0;
    for (int i = rowsWritten; i < end; i++) {
        const LogRow& r = rows[i];
        len += snprintf(buf + len, sizeof(buf) - len, "%lu,%.1f,%.1f,%.1f,%.1f,%u,%u",
                        (unsigned long)r.t_ms, r.x, r.y, r.theta_d10 / 10.0, r.lift_d10 / 10.0,
                        (unsigned)r.claw, (unsigned)r.period);
        for (int k = 0; k < kTempCount; k++) {
            len += snprintf(buf + len, sizeof(buf) - len, ",%u", (unsigned)r.temps[k]);
        }
        len += snprintf(buf + len, sizeof(buf) - len, "\n");
        if (len > (int)sizeof(buf) - 160) { sdWrite(buf, len); len = 0; }
    }
    sdWrite(buf, len);
    rowsWritten = end;
    Brain.Screen.printAt(10, 230, false, "LOG: %s %d rows %d B   ", logName, rowsWritten, bytesWritten);
    flushing = false;
}

static int loggerTask() {
    uint32_t startMs = 0, lastTempMs = 0;
    bool running = false;
    uint8_t temps[kTempCount] = {0};

    while (true) {
        bool enabled = comp && comp->isEnabled();

        if (enabled && !running) {
            running = true;
            printf("[SD] logging started (auto=%d)\n", (int)comp->isAutonomous());
            rowCount = 0;
            loggerBegin();
            startMs = timer::system();
            readTemps(temps);
            lastTempMs = 0;
        } else if (!enabled && running) {
            running = false;
            printf("[SD] disabled, rows=%d\n", rowCount);
            loggerFlush();
        }

        if (running) {
            uint32_t t = timer::system() - startMs;
            if (t - lastTempMs >= (uint32_t)kTempEveryMs) {
                readTemps(temps);
                lastTempMs = t;
            }
            sampleRow(t, temps);
            if (rowCount - rowsWritten >= 40) loggerFlush();
            uint32_t elapsed = (timer::system() - startMs) - t;
            this_thread::sleep_for(elapsed < (uint32_t)kSampleMs ? kSampleMs - elapsed : 1);
        } else {
            this_thread::sleep_for(100);
        }
    }
}

void loggerInit(competition& c) {
    comp = &c;
    static thread loggerThread(loggerTask);
}
