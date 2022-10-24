/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       williamgiles                                              */
/*    Created:      Tue May 03 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       William Giles                                             */
/*    Created:      Fri Jul 31 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// L3                   motor         20              
// L2                   motor         19              
// R3                   motor         11              
// R2                   motor         12              
// Controller1          controller                    
// Inertial             inertial      16              
// L1                   motor         18              
// R1                   motor         13              
// roller               motor         14              
// Launcher             digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "iostream"
#include "vex.h"
#include <cmath>
#include <math.h>
#include <string>

using namespace vex;
competition Competition;

float move_max = 1;
float move_min = .3;
float spin = .8;
float amp_ = 2.5;

bool move_rev = false;

float WheelCircumference = (101.6 * 3.14159);

double gyroScale = .98;

float move = move_max;

int dead_zone = 10;

#define NONE 2

#define btnNONE 0
#define btnUP 1
#define btnDOWN 2
#define btnLEFT 3
#define btnRIGHT 4
#define btnA 5
#define btnB 6
#define btnX 7
#define btnY 8
#define btnR1 9
#define btnR2 10
#define btnL1 11
#define btnL2 12
#define bounceDelay 20
#define refreshDelay 150
#define screenTextWidth 16
#define screenTextHeight 3
#define maxMenus 3
#define modeDisabled 0
#define modeDriver 1
#define modeAuton 2
#define modeError 3
#define RED 0
#define BLUE 1
#define FRONT 0
#define BACK 1
#define SKILLS 2
#define RC 0
#define TANK 1
#define RC2 2
#define AUTON_COLOR 0
#define AUTON_TYPE 1
#define AUTON_DRIVE 2
#define CLOSED 0
#define OPEN 1

#define IN false
#define OUT true

bool screen = false;

bool isCursorOn = false;
int tempStatus = 0;
int autoTempStatus = 0;

int power_usage = 0;

float F_Left;
float B_Left;
float F_Right;
float B_Right;

int Ch1;
int Ch2;
int Ch3;
int Ch4;

bool yeeted = false;

// bool flywheelState = false;
// bool flywheelDir = IN;
// int flywheelSpeed = 100;

bool expandState = IN;

void delay(int amount_eeeeee) { task::sleep(amount_eeeeee); }

void log(std::string logggggggg) { std::cout << logggggggg << std::endl; }

// motor_group leftGroup(LB1, LB2, LF1, LF2);
// motor_group rightGroup(RB1, RB2, RF1, RF2);

motor_group leftGroup(L1, L2, L3);
motor_group rightGroup(R1, R2, R3);

// motor_group expand(expandL, expandR);

drivetrain driveTrain(leftGroup, rightGroup, 320, 382, 334, mm, 1.5);

// pneumatics claw = pneumatics(Brain.ThreeWirePort.A);

// pneumatics Launcher = pneumatics(Brain.ThreeWirePort.A);
// pneumatics spiner = pneumatics(Brain.ThreeWirePort.B);


double map(double x, double in_min, double in_max, double out_min,
           double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double getDir() { return Inertial.rotation() / gyroScale; }

void drive(double deeeez, int speed) {
  driveTrain.driveFor(directionType::fwd, deeeez, distanceUnits::mm, speed,
                      velocityUnits::pct);
}

void drivePd(double deeeez, double sp) {
  if (Inertial.installed()) {
    // Inertial.resetRotation();
    leftGroup.resetPosition();
    rightGroup.resetPosition();

    int rev = 1;
    if (deeeez < 0)
      rev = -1;

    // int average = (left.position(deg) + FB2.position(deg) +
    //                BackLeft.position(deg) + FrontRight.position(deg) +
    //                MidRight.position(deg) + BackRight.position(deg)) /
    //               6;

    int average = (leftGroup.position(deg) + rightGroup.position(deg)) / 2;
    average *= rev;

    int offsetRot = getDir();

    int dt = 20;

    double target = (deeeez / 320) * 360;

    double error = target - average;
    double kP = 1;
    double kD = 20;
    double prevError = error;
    while (std::abs(error) > 5) {
      // average = (FrontLeft.position(deg) + MidLeft.position(deg) +
      //            BackLeft.position(deg) + FrontRight.position(deg) +
      //            MidRight.position(deg) + BackRight.position(deg)) /
      //           6;

      average = (leftGroup.position(deg) + rightGroup.position(deg)) / 2;
      average *= rev;

      error = target - average;
      double derivative = (error - prevError) / dt;
      double percent = kP * error + kD * derivative;
      percent = percent > 100 ? 100 : percent;

      int left_ = (percent * sp) - ((getDir() - offsetRot) * 2 * rev);
      int right_ = (percent * sp) - ((getDir() - offsetRot) * 2 * rev);

      leftGroup.spin(directionType::fwd, left_, pct);
      rightGroup.spin(directionType::fwd, right_, pct);
      vex::task::sleep(dt);
      prevError = error;

      log(error);
    }
    leftGroup.stop();
    rightGroup.stop();
  } else {
    log("fuk");
  }
}

void pTurn(double degrees) // p loop
{
  if (Inertial.installed()) {
    Inertial.resetRotation();

    int dt = 20;
    double target = degrees;
    double error = target - getDir();
    double kP = .6;
    while (std::abs(error) > 1) {
      error = target - getDir();
      double percent = kP * error + 20 * error / std::abs(error);
      leftGroup.spin(directionType::fwd, percent, pct);
      rightGroup.spin(directionType::rev, percent, pct);
      vex::task::sleep(dt);
    }
    leftGroup.stop();
    rightGroup.stop();
  } else {
    log("fuk");
  }
}

void pdTurn(double degrees, double sp = 1) // pd loop
{
  if (Inertial.installed()) {
    // Inertial.resetRotation();

    int dt = 50;
    double target = degrees;
    double error = target - getDir();
    double kP = .5;
    double kD = 5;
    double kI = 1000;
    double prevError = error;
    double intergral = 0;
    while (std::abs(error) > .5) {
      error = target - getDir();
      intergral += error;
      double derivative = (error - prevError) / dt;
      // double percent = kP * error + kI * intergral + kD * derivative;
      double percent = error * kP + intergral / kI;

      leftGroup.spin(directionType::fwd, percent * sp, pct);
      rightGroup.spin(directionType::rev, percent * sp, pct);
      vex::task::sleep(dt);
      prevError = error;
    }
    leftGroup.stop();
    rightGroup.stop();
  } else {
    log("fuk");
  }
}

//===============================================MADNESS===============================================

std::string robotStatus[4] = {"Robot Disabled", "Driver Control",
                              "Auton Control ", "MODE ERROR    "};

int maxMenusIndex[maxMenus] = {2, 9, 3};
int configuration[maxMenus] = {0, 1, 0};

std::string menuTypes[maxMenus] = {"Color: ", "Option: ", "Drive: "};

std::string menuOptions[maxMenus][9] = {
    {"Red", "Blue"},
    {"Skills", "1", "2", "3", "4", "5", "G1", "G2", "G3"},
    {"RC", "Tank", "RC2"}};

int keyPressedRaw() {
  if (Controller1.ButtonUp.pressing() == true)
    return btnUP;
  if (Controller1.ButtonDown.pressing() == true)
    return btnDOWN;
  if (Controller1.ButtonLeft.pressing() == true)
    return btnLEFT;
  if (Controller1.ButtonRight.pressing() == true)
    return btnRIGHT;
  if (Controller1.ButtonA.pressing() == true)
    return btnA;
  if (Controller1.ButtonB.pressing() == true)
    return btnB;
  if (Controller1.ButtonX.pressing() == true)
    return btnX;
  if (Controller1.ButtonY.pressing() == true)
    return btnY;
  if (Controller1.ButtonR1.pressing() == true)
    return btnR1;
  if (Controller1.ButtonR2.pressing() == true)
    return btnR2;
  if (Controller1.ButtonL1.pressing() == true)
    return btnL1;
  if (Controller1.ButtonL2.pressing() == true)
    return btnL2;
  else
    return btnNONE;
}
int keyPressed() {
  int noBounceKey = keyPressedRaw();
  task::sleep(bounceDelay);
  if (noBounceKey == keyPressedRaw()) {
    return noBounceKey;
  } else {
    return btnNONE;
  }
}
void clearLine(int l_row) {
  if (isCursorOn == true) {
    Controller1.Screen.setCursor(l_row + 1, 2);
    Controller1.Screen.print("               ");
  } else
    Controller1.Screen.clearLine();
}
void clearScreen(void) { Controller1.Screen.clearScreen(); }

void print(std::string text, int row, int col) {
  clearLine(row);
  if (isCursorOn == true) {
    Controller1.Screen.setCursor(row + 1, col + 2);
  } else {
    Controller1.Screen.setCursor(row + 1, col + 1);
  }
  Controller1.Screen.print(text.c_str());
}

void print2(std::string text, int row, int col) {
  Controller1.Screen.setCursor(row, 0);
  Controller1.Screen.print("               ");
  Controller1.Screen.setCursor(row, col);
  Controller1.Screen.print(text.c_str());
}

void selector(int row) {
  isCursorOn = true;
  for (int i = 1; i <= screenTextHeight; i++) {
    Controller1.Screen.setCursor(i, 1);
    Controller1.Screen.print("|");
  }
  int showCursor = (row % screenTextHeight) + 1;
  Controller1.Screen.setCursor(showCursor, 1);
  Controller1.Screen.print(">");
}
void notificationHUD(std::string str) {
  clearLine(0);
  print(str, 0, 0);
}
int currStatus() {
  int status;
  if (Competition.isEnabled()) {
    if (Competition.isAutonomous()) {
      status = modeAuton;
    } else if (Competition.isDriverControl()) {
      status = modeDriver;
    } else {
      status = modeError;
    }
  } else {
    status = modeDisabled;
  }
  return status;
}
void statusHUD() {
  std::string temp;
  print(robotStatus[currStatus()], 1, 0);
  temp = menuOptions[0][configuration[0]] + " " +
         menuOptions[1][configuration[1]] + " " +
         menuOptions[2][configuration[2]] + " ";
  print(temp, 2, 0);
}
void displayMenu(int currRow, int configuration[]) {
  std::string temp;
  selector(currRow);
  for (int i = 0; i < screenTextHeight; i++) {
    temp = menuTypes[i] + menuOptions[i][configuration[i]];
    print(temp, i, 0);
  }
}
int getValues(int wantConfig) {
  if (wantConfig > maxMenus) {
    return NULL;
  } else {
    return configuration[wantConfig];
  }
}
void menuCONFIG() {
  isCursorOn = true;
  int currCursorMenu = 0;
  int currCursorOptions = 0;
  bool isAutonSelectScreen = true;
  while (isAutonSelectScreen == true) {
    displayMenu(currCursorMenu, configuration);
    bool isValidButton = false;
    while (isValidButton == false) {
      switch (keyPressed()) {
      case (btnUP):
        isValidButton = true;
        currCursorMenu--;
        currCursorMenu = (currCursorMenu + maxMenus) % maxMenus;
        currCursorOptions = configuration[currCursorMenu];
        break;
      case (btnDOWN):
        isValidButton = true;
        currCursorMenu++;
        currCursorMenu = (currCursorMenu + maxMenus) % maxMenus;
        currCursorOptions = configuration[currCursorMenu];
        break;
      case (btnLEFT):
        isValidButton = true;
        currCursorOptions--;
        currCursorOptions =
            (currCursorOptions + maxMenusIndex[currCursorMenu]) %
            maxMenusIndex[currCursorMenu];
        configuration[currCursorMenu] = currCursorOptions;
        break;
      case (btnRIGHT):
        isValidButton = true;
        currCursorOptions++;
        currCursorOptions =
            (currCursorOptions + maxMenusIndex[currCursorMenu]) %
            maxMenusIndex[currCursorMenu];
        configuration[currCursorMenu] = currCursorOptions;
        break;
      case (btnA):
        isValidButton = true;
        isAutonSelectScreen = false;
        isCursorOn = false;
        break;
      default:
        break;
      }
    }
  }
}

//===============================================END OF
// MADNESS===============================================

void calabrate(void) {

  // FrontLeft.resetRotation();
  // FrontRight.resetRotation();
  // BackLeft.resetRotation();
  // BackRight.resetRotation();
  // MidLeft.resetPosition();
  // MidRight.resetPosition();

  // ArmLeft.resetPosition();
  // ArmRight.resetPosition();

  Inertial.resetRotation();

  L1.resetRotation();
  L2.resetRotation();
  L3.resetRotation();

  R1.resetRotation();
  R2.resetRotation();
  R3.resetRotation();

  roller.resetRotation();
}

void Move(int Ch1_, int Ch3_, int Ch4_) {
  Ch1 = Ch1_;
  Ch3 = Ch3_;
  Ch4 = Ch4_;

  F_Left = Ch3 + Ch4;
  // B_Left = Ch3 + Ch4;
  F_Right = Ch3 - Ch4;
  // B_Right = Ch3 - Ch4;

  leftGroup.spin(directionType::fwd, F_Left, velocityUnits::pct);
  rightGroup.spin(directionType::fwd, F_Right, velocityUnits::pct);

  // FrontLeft.spin(directionType::fwd, F_Left, velocityUnits::pct);
  // BackLeft.spin(directionType::fwd, F_Left, velocityUnits::pct);
  // MidLeft.spin(directionType::fwd, F_Left, velocityUnits::pct);

  // FrontRight.spin(directionType::fwd, F_Right, velocityUnits::pct);
  // BackRight.spin(directionType::fwd, F_Right, velocityUnits::pct);
  // MidRight.spin(directionType::fwd, F_Right, velocityUnits::pct);
}

void pre_auton(void) {
  menuCONFIG();

  Inertial.calibrate(2000);
  vex::task::sleep(2000);

  notificationHUD("PRE: DONE");

  Controller1.rumble(".");
}

void auton1() {
  // win

  Move(0, -50, 0);
  delay(1000);
  Move(0, 50, 0);
  delay(1000);
  Move(0, 0, 0);
}

void auton2() {
  // armGroup.rotateTo(-200, deg, 100, velocityUnits::pct);

  // delay(200);

  // armGroup.rotateTo(-1400, deg, 100, velocityUnits::pct);
}

void auton3() {}

void auton4() {}

void auton5() {}

void autonG1(void) {
  // win
  drivePd(1000, .5);

  delay(200);

  pdTurn(180);

  delay(200);

  drivePd(500, .5);

  delay(200);

  pdTurn(0);

  delay(200);

  drivePd(-500, .5);
}

void autonG2(void) {
  // win

  // delay(200);

  // pdTurn(180);

  // delay(200);
}

void autonG3(void) {}

void autonSkills() {}

void auton(void) {
  // FrontLeft.setStopping(brake);
  // FrontRight.setStopping(brake);
  // BackLeft.setStopping(brake);
  // BackRight.setStopping(brake);
  // MidLeft.setStopping(brake);
  // MidRight.setStopping(brake);

  leftGroup.setStopping(hold);
  rightGroup.setStopping(hold);

  // armGroup.setStopping(hold);

  // driveTrain.setStopping(coast);

  if (configuration[1] == 0) { // Skills
    autonSkills();
  } else if (configuration[1] == 1) {
    auton1();
  } else if (configuration[1] == 2) {
    auton2();
  } else if (configuration[1] == 3) {
    auton3();
  } else if (configuration[1] == 4) {
    auton4();
  } else if (configuration[1] == 5) {
    auton5();
  } else if (configuration[1] == 6) {
    autonG1();
  } else if (configuration[1] == 7) {
    autonG2();
  } else if (configuration[1] == 8) {
    autonG3();
  }

  notificationHUD("Auton: DONE");
  Controller1.rumble(".");
}

void RCDrive(void) {
  Ch1 = Controller1.Axis1.position(percent) * move;
  Ch3 = Controller1.Axis3.position(percent) * move;
  Ch4 = Controller1.Axis4.position(percent) * move;
  // Ch4 = Ch4 * spin;

  // Ch1 = 0.95 * Ch1 + 0.05 * Controller1.Axis1.position(percent);
  // Ch3 = 0.95 * Ch3 + 0.05 * Controller1.Axis3.position(percent);
  // Ch4 = 0.95 * Ch4 + 0.05 * Controller1.Axis4.position(percent);

  if (Ch1 < dead_zone && Ch1 > -dead_zone) {
    Ch1 = 0;
  }
  if (Ch2 < dead_zone && Ch2 > -dead_zone) {
    Ch2 = 0;
  }
  if (Ch3 < dead_zone && Ch3 > -dead_zone) {
    Ch3 = 0;
  }
  if (Ch4 < dead_zone && Ch4 > -dead_zone) {
    Ch4 = 0;
  }

  F_Left = Ch3 + Ch4;
  // B_Left = Ch3 + Ch4;
  F_Right = Ch3 - Ch4;
  // B_Right = Ch3 - Ch4;

  F_Left = F_Left > 100 ? 100 : F_Left;
  F_Right = F_Right > 100 ? 100 : F_Right;

  // log("=======");

  // log(F_Left);

  F_Left = 15 * (F_Left / 100);
  F_Right = 15 * (F_Right / 100);

  // log(F_Left);

  // FrontLeft.spin(directionType::fwd, F_Left, velocityUnits::pct);
  // BackLeft.spin(directionType::fwd, F_Left, velocityUnits::pct);
  // MidLeft.spin(directionType::fwd, F_Left, velocityUnits::pct);

  // leftGroup.spin(directionType::fwd, F_Left, velocityUnits::pct);

  // FrontRight.spin(directionType::fwd, F_Right, velocityUnits::pct);
  // BackRight.spin(directionType::fwd, F_Right, velocityUnits::pct);
  // MidRight.spin(directionType::fwd, F_Right, velocityUnits::pct);

  // rightGroup.spin(directionType::fwd, F_Right, velocityUnits::pct);

  // rightGroup.spin(directionType::fwd, 14, voltageUnits::volt);

  if (move_rev) {
    leftGroup.spin(directionType::rev, F_Right, voltageUnits::volt);
    rightGroup.spin(directionType::rev, F_Left, voltageUnits::volt);

  } else {
    leftGroup.spin(directionType::fwd, F_Left, voltageUnits::volt);
    rightGroup.spin(directionType::fwd, F_Right, voltageUnits::volt);
  }
}

void RCDrive2(void) {
  // hmmm
}

void tankDrive(void) {
  // hmmm
}

void user(void) {

  while (keyPressed() != btnNONE)
    ;

  leftGroup.setStopping(hold);
  rightGroup.setStopping(hold);

  roller.setStopping(hold);


  while (1) {

    if (keyPressedRaw() == btnUP) {
      move = move_max;
    }
    if (keyPressedRaw() == btnDOWN) {
      move = move_min;
    }

    if (keyPressedRaw() == btnRIGHT) {
      calabrate();
      auton();
    }

    switch (getValues(AUTON_DRIVE)) {
    case RC:
      RCDrive();
      break;
    case TANK:
      tankDrive();
      break;
    case RC2:
      RCDrive2();
      break;
    default:
      break;
    }

    if (Controller1.ButtonX.pressing()) {
      move_rev = !move_rev;

      while (Controller1.ButtonX.pressing())
        ;
    }

    // if (Controller1.ButtonY.pressing()) {
    //   if(yeeted)  YEET.rotateFor(180, deg, 100, velocityUnits::pct);
    //   else YEET.rotateFor(-180, deg, 100, velocityUnits::pct);

    //   yeeted = !yeeted;

    //   while (Controller1.ButtonY.pressing())
    //     ;
    // }


     if (Controller1.ButtonY.pressing()) {
      if(yeeted) Launcher.set(true);
      else  Launcher.set(false);

      yeeted = !yeeted;

      while (Controller1.ButtonY.pressing())
        ;
    }

    if (Controller1.ButtonR1.pressing()) {
      roller.spin(directionType::fwd, 14, voltageUnits::volt);
    } else if (Controller1.ButtonR2.pressing()) {
      roller.spin(directionType::rev, 14, voltageUnits::volt);
    } else {
      roller.stop();
    }

  

    // looooooooooooooooooooooooooooooooooooooop
  }
}

char buf[512];

int main() {

  vexcodeInit();

  calabrate();

  pre_auton();

  Competition.autonomous(auton);
  Competition.drivercontrol(user);

  if (getValues(AUTON_COLOR) == RED) {
    Brain.Screen.setFillColor(red);
  } else if (getValues(AUTON_COLOR) == BLUE) {
    Brain.Screen.setFillColor(blue);
  }

  Brain.Screen.drawRectangle(-10, -10, 500, 500);

  Brain.Screen.setPenColor(color::white);
  Brain.Screen.setFont(prop60);
  Brain.Screen.printAt(150, 120, "2903 S");

  while (1) {
    power_usage = 0;
    power_usage += L1.power(watt);
    power_usage += L2.power(watt);
    power_usage += L3.power(watt);

    power_usage += R1.power(watt);
    power_usage += R2.power(watt);
    power_usage += R3.power(watt);


    if (Brain.Screen.pressing() && screen == false) {
      screen = true;
      Brain.Screen.clearScreen();
    }

    if (screen) {
      Brain.Screen.setFillColor(blue);
      Brain.Screen.drawRectangle(5, 20, 60, 30);

      Brain.Screen.setPenColor(color::white);
      Brain.Screen.setFont(prop20);
      Brain.Screen.printAt(25, 39, "Back");

      Brain.Screen.setCursor(2, 10);
      Brain.Screen.print(power_usage);
      Brain.Screen.print("  ");
      // Brain.Screen.setCursor(2, 15);
      // Brain.Screen.print(ArmRight.position(deg));
      // Brain.Screen.print("  ");

      // Brain.Screen.setCursor(7, 25);
      // Brain.Screen.print(armGroup.position(deg));
      // Brain.Screen.print("  ");

      Brain.Screen.setCursor(2, 25);
      Brain.Screen.print(getDir());
      Brain.Screen.print("  ");
      Brain.Screen.setCursor(2, 30);
      Brain.Screen.print(Inertial.rotation());
      Brain.Screen.print("  ");

      // Brain.Screen.setCursor(4, 10);
      // Brain.Screen.print(FrontLeft.current());
      // Brain.Screen.print("  ");
      // Brain.Screen.setCursor(4, 15);
      // Brain.Screen.print(FrontRight.current());
      // Brain.Screen.print("  ");
      // Brain.Screen.setCursor(4, 20);
      // Brain.Screen.print(BackLeft.current());
      // Brain.Screen.print("  ");
      // Brain.Screen.setCursor(4, 25);
      // Brain.Screen.print(BackRight.current());
      // Brain.Screen.print("  ");

      // Brain.Screen.setCursor(5, 10);
      // Brain.Screen.print(FrontLeft.position(rotationUnits::rev));
      // Brain.Screen.print("  ");
      // Brain.Screen.setCursor(5, 15);
      // Brain.Screen.print(FrontRight.position(rotationUnits::rev));
      // Brain.Screen.print("  ");
      // Brain.Screen.setCursor(5, 20);
      // Brain.Screen.print(BackLeft.position(rotationUnits::rev));
      // Brain.Screen.print("  ");
      // Brain.Screen.setCursor(5, 25);
      // Brain.Screen.print(BackRight.position(rotationUnits::rev));
      // Brain.Screen.print("  ");

      if (Brain.Screen.pressing()) {

        int xPos = Brain.Screen.xPosition();
        int yPos = Brain.Screen.yPosition();
        while (Brain.Screen.pressing())
          ;

        if (xPos > 5 && xPos < 5 + 60 && yPos > 20 && yPos < 20 + 30) {
          screen = false;
          Brain.Screen.clearScreen();
        }
      }
    }

    if (tempStatus != currStatus()) { //  || autoTempStatus != automatic
      statusHUD();
      tempStatus = currStatus();
      // autoTempStatus = automatic;
    }
    delay(200);
  }
}
