#include "brainUI.h"

int motorStatusTimer = 0;
bool frontLeftStatus = frontLeftDrive.installed();
bool midLeftStatus = midLeftDrive.installed();
bool backLeftStatus = backLeftDrive.installed();
bool frontRightStatus = frontRightDrive.installed();
bool midRightStatus = midRightDrive.installed();
bool backRightStatus = backRightDrive.installed();
bool lift1Status = lift1.installed();
bool lift2Status = lift2.installed();
bool lift3Status = lift3.installed();
bool odomStatus = odomPod.installed();
bool intertialStatus = inertialSensor.installed();
bool liftStatus = liftSensor.installed();
bool calibrated = false;

void drawButton(int x, int y, int w, int h, std::string t, int destination) {
    Brain.Screen.setFillColor(black);
    Brain.Screen.setPenColor(white);
    Brain.Screen.drawRectangle(x, y, w, h);

    int textWidth = Brain.Screen.getStringWidth(t.c_str());
    int textHeight = Brain.Screen.getStringHeight(t.c_str());
    int textX = x + (w - textWidth) / 2;
    int textY = y + (h + textHeight) / 2;
    Brain.Screen.printAt(textX, textY, false, t.c_str());

    if (Brain.Screen.pressing()) {
        int touchX = Brain.Screen.xPosition();
        int touchY = Brain.Screen.yPosition();

        if (touchX >= x && touchX <= x + w && touchY >= y && touchY <= y + h) {
            screen = destination;
            Brain.Screen.clearScreen();
            return;
        }
    }
}

void drawAutonButton(int x, int y, int w, int h, std::string t, int autonNum, color textColor = white) {
    Brain.Screen.setFillColor(black);
    
    if (currentAuton == autonNum) {
        Brain.Screen.setPenColor(blue);
        Brain.Screen.setPenWidth(4);
    } else {
        Brain.Screen.setPenColor(white);
        Brain.Screen.setPenWidth(1);
    }
    
    Brain.Screen.drawRectangle(x, y, w, h);
    Brain.Screen.setPenWidth(1);

    int textWidth = Brain.Screen.getStringWidth(t.c_str());
    int textHeight = Brain.Screen.getStringHeight(t.c_str());
    int textX = x + (w - textWidth) / 2;
    int textY = y + (h + textHeight) / 2;
    
    Brain.Screen.setPenColor(textColor);
    Brain.Screen.printAt(textX, textY, false, t.c_str());

    if (Brain.Screen.pressing()) {
        int touchX = Brain.Screen.xPosition();
        int touchY = Brain.Screen.yPosition();

        if (touchX >= x && touchX <= x + w && touchY >= y && touchY <= y + h) {
            currentAuton = autonNum;
            
            // Update controller display
            Controller.Screen.clearScreen();
            Controller.Screen.setCursor(1, 1);
            if (currentAuton == -1) {
                Controller.Screen.print("Auton: Drive Forward");
            } else if (currentAuton == 0) {
                Controller.Screen.print("Auton: Loader, toggle");
            } else if (currentAuton == 1) {
                Controller.Screen.print("Auton: Non-Loader, toggle);
            } else if (currentAuton == 2) {
                Controller.Screen.print("Auton: Loader, no toggle");
            } else if (currentAuton == 3) {
                Controller.Screen.print("Auton: Non-loader, no toggle");
            }
            
            wait(100, msec);
            return;
        }
    }
}

void brainUI(void){

  motorStatusTimer+=5;

  if (motorStatusTimer>=500){ 
    frontLeftStatus = frontLeftDrive.installed();
    midLeftStatus = midLeftDrive.installed();
    backLeftStatus = backLeftDrive.installed();
    frontRightStatus = frontRightDrive.installed();
    midRightStatus = midRightDrive.installed();
    backRightStatus = backRightDrive.installed();
    lift1Status = lift1.installed();
    lift2Status = lift2.installed();
    lift3Status = lift3.installed();
    odomStatus = odomPod.installed();
    intertialStatus = inertialSensor.installed();
    liftStatus = liftSensor.installed();
  }

  Brain.Screen.clearScreen();

  switch(screen){
    case 0:
      Brain.Screen.setFillColor(black);
      Brain.Screen.setPenColor(white);

      drawButton(35, 40, 200, 160, "Auton", 1);
      drawButton(245, 40, 200, 160, "Status Check", 2);
      break;

    case 1:
      Brain.Screen.setFillColor(black);
      Brain.Screen.setPenColor(white);
      Brain.Screen.setFont(prop30);

      drawAutonButton(10, 10, 460, 42, "Drive Forward", -1, red);
      drawAutonButton(10, 56, 460, 42, "Loader, Toggle", 0, white);
      drawAutonButton(10, 102, 460, 42, "Non-Loader, Toggle", 1, white);
      drawAutonButton(10, 148, 460, 42, "Loader, no toggle" 2, white);
      drawAutonButton(10, 194, 460, 42, "Non-Loader, No toggle", 3, white);
      
      Brain.Screen.setFont(prop20);
      drawButton(405, 212, 60, 20, "Back", 0);
      break;

    case 2:
      Brain.Screen.setFillColor(black);
      Brain.Screen.setPenColor(white);
      Brain.Screen.setFont(prop20);

      Brain.Screen.setPenColor(frontLeftStatus ? white : red);
      Brain.Screen.printAt(10, 15, false, "Front Left (P4): %s", frontLeftStatus ? "OK" : "ERR");
      if (frontLeftStatus) {
        double flTemp = frontLeftDrive.temperature(celsius);
        Brain.Screen.setPenColor(flTemp > 50 ? (flTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 15, false, "%.1fC", flTemp);
      }

      Brain.Screen.setPenColor(midLeftStatus ? white : red);
      Brain.Screen.printAt(10, 30, false, "Middle Left (P6): %s", midLeftStatus ? "OK" : "ERR");
      if (midLeftStatus) {
        double mlTemp = midLeftDrive.temperature(celsius);
        Brain.Screen.setPenColor(mlTemp > 50 ? (mlTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 30, false, "%.1fC", mlTemp);
      }

      Brain.Screen.setPenColor(backLeftStatus ? white : red);
      Brain.Screen.printAt(10, 45, false, "Back Left (P5): %s", backLeftStatus ? "OK" : "ERR");
      if (backLeftStatus) {
        double blTemp = backLeftDrive.temperature(celsius);
        Brain.Screen.setPenColor(blTemp > 50 ? (blTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 45, false, "%.1fC", blTemp);
      }

      Brain.Screen.setPenColor(frontRightStatus ? white : red);
      Brain.Screen.printAt(10, 60, false, "Front Right (P8): %s", frontRightStatus ? "OK" : "ERR");
      if (frontRightStatus) {
        double frTemp = frontRightDrive.temperature(celsius);
        Brain.Screen.setPenColor(frTemp > 50 ? (frTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 60, false, "%.1fC", frTemp);
      }

      Brain.Screen.setPenColor(midRightStatus ? white : red);
      Brain.Screen.printAt(10, 75, false, "Middle Right (P10): %s", midRightStatus ? "OK" : "ERR");
      if (midRightStatus) {
        double mrTemp = middleRightDrive.temperature(celsius);
        Brain.Screen.setPenColor(mrTemp > 50 ? (mrTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 75, false, "%.1fC", mrTemp);
      }

      Brain.Screen.setPenColor(backRightStatus ? white : red);
      Brain.Screen.printAt(10, 90, false, "Back Right (P9): %s", backRightStatus ? "OK" : "ERR");
      if (backRightStatus) {
        double brTemp = backRightDrive.temperature(celsius);
        Brain.Screen.setPenColor(brTemp > 50 ? (brTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 90, false, "%.1fC", brTemp);
      }

      Brain.Screen.setPenColor(lift1Status ? white : red);
      Brain.Screen.printAt(10, 105, false, "Lift1 (P1): %s", lift1Status ? "OK" : "ERR");
      if (lift1Status) {
        double lbtTemp = lowBlockTrack.temperature(celsius);
        Brain.Screen.setPenColor(lbtTemp > 50 ? (lbtTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 105, false, "%.1fC", lbtTemp);
      }

      Brain.Screen.setPenColor(lift2Status ? white : red);
      Brain.Screen.printAt(10, 120, false, "Lift2 (P2): %s", highBlockTrackStatus ? "OK" : "ERR");
      if (highBlockTrackStatus) {
        double hbtTemp = highBlockTrack.temperature(celsius);
        Brain.Screen.setPenColor(hbtTemp > 50 ? (hbtTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 120, false, "%.1fC", hbtTemp);
      }

      Brain.Screen.setPenColor(lift3Status ? white : red);
      Brain.Screen.printAt(10, 120, false, "Lift3 (P3): %s", lift3Status ? "OK" : "ERR");
      if (lift3Status) {
        double hbtTemp = highBlockTrack.temperature(celsius);
        Brain.Screen.setPenColor(hbtTemp > 50 ? (hbtTemp > 55 ? red : orange) : white);
        Brain.Screen.printAt(420, 120, false, "%.1fC", hbtTemp);
      }

      Brain.Screen.setPenColor(odomStatus ? white : red);
      Brain.Screen.printAt(10, 135, false, "Odom Pod (P20): %s", odomStatus ? "OK" : "ERR");
      
      Brain.Screen.setPenColor(liftStatus ? white : red);
      Brain.Screen.printAt(10, 135, false, "Lift Height Sensor (P1): %s", liftSensor ? "OK" : "ERR");

      Brain.Screen.setPenColor(inertialStatus ? white : red);
      Brain.Screen.printAt(240, 15, false, "IMU (P19): %s", inertialStatus ? "OK" : "ERR");

      Brain.Screen.setPenColor(white);
      drawButton(5, 212, 60, 20, "Back", 0);
      break;

    }

