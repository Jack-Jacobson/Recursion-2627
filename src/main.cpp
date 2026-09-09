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

namespace {
    const double deadbandThreshold = 5.0;
    bool useOneStickDrive = true;
    bool wasTopRightTriggerPressed = false;

    double applyDeadband(double value) {
        if (value > -deadbandThreshold && value < deadbandThreshold) {
            return 0.0;
        }
        return value;
    }

    double clampToPercent(double value) {
        if (value > 100.0) {
            return 100.0;
        }
        if (value < -100.0) {
            return -100.0;
        }
        return value;
    }

    void setDriveCoast() {
        frontLeftDrive.setStopping(vex::brakeType::coast);
        midLeftDrive.setStopping(vex::brakeType::coast);
        backLeftDrive.setStopping(vex::brakeType::coast);
        frontRightDrive.setStopping(vex::brakeType::coast);
        midRightDrive.setStopping(vex::brakeType::coast);
        backRightDrive.setStopping(vex::brakeType::coast);
    }

    void spinDriveMotor(motor& driveMotor, double percentOutput) {
        if (percentOutput == 0.0) {
            driveMotor.stop(vex::brakeType::coast);
            return;
        }

        driveMotor.spin(vex::directionType::fwd, percentOutput, vex::velocityUnits::pct);
    }

    void updateDriveModeToggle() {
        const bool isTopRightTriggerPressed = Controller.ButtonR1.pressing();

        if (isTopRightTriggerPressed && !wasTopRightTriggerPressed) {
            useOneStickDrive = !useOneStickDrive;
        }

        wasTopRightTriggerPressed = isTopRightTriggerPressed;
    }
}

void oneStickDrive() {
    const double turnInput = -Controller.Axis2.position(vex::percent);
    const double forwardInput = -Controller.Axis1.position(vex::percent);

    double turn = applyDeadband(turnInput);
    double forward = applyDeadband(forwardInput);

    if (turn > 0.0) {
        forward *= -1.0;
    }

    double leftPercent = forward + turn;
    double rightPercent = forward - turn;

    leftPercent = clampToPercent(leftPercent);
    rightPercent = clampToPercent(rightPercent);

    spinDriveMotor(frontLeftDrive, leftPercent);
    spinDriveMotor(midLeftDrive, leftPercent);
    spinDriveMotor(backLeftDrive, leftPercent);

    spinDriveMotor(frontRightDrive, rightPercent);
    spinDriveMotor(midRightDrive, rightPercent);
    spinDriveMotor(backRightDrive, rightPercent);
}

void twoStickDrive() {
    const double leftInput = -Controller.Axis2.position(vex::percent);
    const double rightInput = -Controller.Axis3.position(vex::percent);

    double leftPercent = applyDeadband(leftInput);
    double rightPercent = applyDeadband(rightInput);

    leftPercent = clampToPercent(leftPercent);
    rightPercent = clampToPercent(rightPercent);

    spinDriveMotor(frontLeftDrive, leftPercent);
    spinDriveMotor(midLeftDrive, leftPercent);
    spinDriveMotor(backLeftDrive, leftPercent);

    spinDriveMotor(frontRightDrive, rightPercent);
    spinDriveMotor(midRightDrive, rightPercent);
    spinDriveMotor(backRightDrive, rightPercent);
}

int main() {
    setDriveCoast();

    while (true) {
        updateDriveModeToggle();

        if (useOneStickDrive) {
            oneStickDrive();
        } else {
            twoStickDrive();
        }
    }
}
