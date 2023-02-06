/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       williamgiles                                              */
/*    Created:      Tue May 03 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// L2                   motor         9
// R2                   motor         2
// Controller1          controller
// Inertial             inertial      16
// L1                   motor         10
// R1                   motor         1
// roller               motor         6
// Launcher             digital_out   A
// back                 digital_out   B
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "PID.h"
#include "iostream"
#include "vex.h"
#include <cmath>
#include <math.h>
#include <string>

using namespace vex;
competition Competition;

void log(std::string logggggggg) { std::cout << logggggggg << std::endl; }

#define logg(x) std::cout << x << std::endl

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

#define UP false
#define DOWN true

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

int cataROT = 0;
bool cataLast = false;

void delay(int amount_eeeeee) { task::sleep(amount_eeeeee); }

// motor_group leftGroup(LB1, LB2, LF1, LF2);
// motor_group rightGroup(RB1, RB2, RF1, RF2);

// motor_group leftGroup(L1, L2, L3);
// motor_group rightGroup(R1, R2, R3);

// pneumatics claw = pneumatics(Brain.ThreeWirePort.A);

// pneumatics Launcher = pneumatics(Brain.ThreeWirePort.A);
// pneumatics spiner = pneumatics(Brain.ThreeWirePort.B);

double map(double x, double in_min, double in_max, double out_min,
           double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double getDir() { return Inertial.rotation() / gyroScale; }

double dirRel = getDir();

double resetDirRel()
{
  dirRel = getDir();
  return dirRel;
}

class virtualHeading
{
private:
  double offset = 0;
  double heading = 0;

public:
  virtualHeading(double offsetI)
  {
    offset = offsetI;
  }

  void set(double offsetI)
  {
    offset = offsetI;
  }

  void setHeading(double headingI)
  {
    heading = headingI;
  }

  double get()
  {
    return getDir() - offset;
  }

  double getHeading()
  {
    return heading;
  }

  void reset()
  {
    offset = 0;
    heading = 0;
  }
};

void drive(double deeeez, int speed)
{
  driveTrain.driveFor(directionType::fwd, deeeez, distanceUnits::mm, speed,
                      velocityUnits::pct);
}

void drivePDNew(double deeeez, double spV, virtualHeading &vh)
{

  leftGroup.resetPosition();
  rightGroup.resetPosition();

  // spV = 50;

  // PID pid(1, .2, .09);
  PID pid(.3, 0.008, 0.01, spV, -spV);

  PID pidTheta(1.5, 0.005, 0.01, 100, -100);

  double target = (deeeez / 320) * 360;

  pid.setTarget(target);
  pidTheta.setTarget(vh.getHeading());

  double theta;
  double pidOut;
  double mSpeed;
  double tolerance = 2;
  double min2 = 5;

  while (1)
  {
    int average = (leftGroup.position(deg) + rightGroup.position(deg)) / 2;

    pidOut = pid.calc(pid.target, average);
    theta = pidTheta.calc(pidTheta.target, vh.get());

    if (std::abs(pid.error) < tolerance)
    {
      leftGroup.stop();
      rightGroup.stop();
      break;
    }

    // if (std::abs(pidOut - theta) <= 100)
    //   pidOut = pidOut - theta;
    // else
    //   pidOut = pidOut + theta;

    if (pidOut < min2 && pidOut > -min2)
    {
      if (pidOut < 0)
        pidOut = -min2;
      else
        pidOut = min2;
    }

    mSpeed = 0.60 * mSpeed + 0.40 * pidOut;

    leftGroup.spin(fwd, mSpeed + theta, pct);
    rightGroup.spin(fwd, mSpeed - theta, pct);

    // logg("speed");
    // logg(mSpeed);
    // logg("error");
    // logg(pid.error);

    // logg(theta);
    // this_thread::sleep_for(20);
    delay(pid.dT * 1000);
  }
}

void turnPDNew(double degs, double spV, virtualHeading &vh)
{
  PID pid(.8, 0.008, 0.01, spV, -spV);

  pid.setTarget(degs);

  double pidOut;
  double mSpeed;
  double tolerance = 1;
  double min2 = 5;
  while (1)
  {
    pidOut = pid.calc(pid.target, vh.get());

    // if (std::abs(pid.error) < tolerance * abs(pid.derivative) * .09)
    //   pid.integral = 0;

    if (abs(pid.derivative) < 1 && std::abs(pid.error) < tolerance * 3)
      goto kill;

    if (std::abs(pid.error) < tolerance && abs(pid.derivative) < 10)
    {
    kill:
      leftGroup.stop();
      rightGroup.stop();
      vh.setHeading(degs);
      break;
    }

    if (pidOut < min2 && pidOut > -min2)
    {
      if (pidOut < 0)
        pidOut = -min2;
      else
        pidOut = min2;
    }

    mSpeed = 0.60 * mSpeed + 0.40 * pidOut;

    leftGroup.spin(fwd, mSpeed, pct);
    rightGroup.spin(directionType::rev, mSpeed, pct);

    delay(pid.dT * 1000);
  }
}

void drivePd(double deeeez, double sp)
{
  if (Inertial.installed())
  {
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
    while (std::abs(error) > 5)
    {
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
  }
  else
  {
    log("fuk");
  }
}

void pdTurn(double degrees, double sp = 1) // pd loop
{
  if (Inertial.installed())
  {
    // Inertial.resetRotation();

    int dt = 50;
    double target = degrees;
    double error = target - getDir();
    double kP = .5;
    double kD = 5;
    double kI = 1000;
    double prevError = error;
    double intergral = 0;
    while (std::abs(error) > .5)
    {
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
  }
  else
    log("fuk");
}

void calabrate(void)
{

  Inertial.resetRotation();

  L1.resetPosition();
  L2.resetPosition();

  R1.resetPosition();
  R2.resetPosition();

  roller.resetPosition();
  intake.resetPosition();

  cata.resetPosition();
}

//===============================================MADNESS===============================================

std::string robotStatus[4] = {"Robot Disabled", "Driver Control",
                              "Auton Control ", "MODE ERROR    "};

int maxMenusIndex[maxMenus] = {3, 9, 3};
int configuration[maxMenus] = {0, 3, 0};

std::string menuTypes[maxMenus] = {"Color: ", "Option: ", "Drive: "};

std::string menuOptions[maxMenus][9] = {
    {"Red", "Blue", "no lgbt"},
    {"Skills", "1", "2", "3", "4", "5", "G1", "G2", "G3"},
    {"RC", "Tank", "RC2"}};

int keyPressedRaw()
{
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
int keyPressed()
{
  int noBounceKey = keyPressedRaw();
  task::sleep(bounceDelay);
  if (noBounceKey == keyPressedRaw())
    return noBounceKey;
  else
    return btnNONE;
}
void clearLine(int l_row)
{
  if (isCursorOn == true)
  {
    Controller1.Screen.setCursor(l_row + 1, 2);
    Controller1.Screen.print("               ");
  }
  else
    Controller1.Screen.clearLine();
}
void clearScreen(void) { Controller1.Screen.clearScreen(); }

void print(std::string text, int row, int col)
{
  clearLine(row);
  if (isCursorOn == true)
    Controller1.Screen.setCursor(row + 1, col + 2);
  else
    Controller1.Screen.setCursor(row + 1, col + 1);
  Controller1.Screen.print(text.c_str());
}

void print2(std::string text, int row, int col)
{
  Controller1.Screen.setCursor(row, 0);
  Controller1.Screen.print("               ");
  Controller1.Screen.setCursor(row, col);
  Controller1.Screen.print(text.c_str());
}

void selector(int row)
{
  isCursorOn = true;
  for (int i = 1; i <= screenTextHeight; i++)
  {
    Controller1.Screen.setCursor(i, 1);
    Controller1.Screen.print("|");
  }
  int showCursor = (row % screenTextHeight) + 1;
  Controller1.Screen.setCursor(showCursor, 1);
  Controller1.Screen.print(">");
}
void notificationHUD(std::string str)
{
  clearLine(0);
  print(str, 0, 0);
}
int currStatus()
{
  int status;
  if (Competition.isEnabled())
  {
    if (Competition.isAutonomous())
      status = modeAuton;
    else if (Competition.isDriverControl())
      status = modeDriver;
    else
      status = modeError;
  }
  else

    status = modeDisabled;

  return status;
}
void statusHUD()
{
  std::string temp;
  print(robotStatus[currStatus()], 1, 0);
  temp = menuOptions[0][configuration[0]] + " " +
         menuOptions[1][configuration[1]] + " " +
         menuOptions[2][configuration[2]] + "*";
  print(temp, 2, 0);
}
void displayMenu(int currRow, int configuration[])
{
  std::string temp;
  selector(currRow);
  for (int i = 0; i < screenTextHeight; i++)
  {
    temp = menuTypes[i] + menuOptions[i][configuration[i]];
    print(temp, i, 0);
  }
}
int getValues(int wantConfig)
{
  if (wantConfig > maxMenus)
    return NULL;
  else
    return configuration[wantConfig];
}
void menuCONFIG()
{
  isCursorOn = true;
  int currCursorMenu = 0;
  int currCursorOptions = 0;
  bool isAutonSelectScreen = true;
  while (isAutonSelectScreen == true)
  {
    displayMenu(currCursorMenu, configuration);
    bool isValidButton = false;
    while (isValidButton == false)
    {
      switch (keyPressed())
      {
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

void Move(int Ch1_, int Ch3_, int Ch4_)
{
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

void pre_auton(void)
{
  menuCONFIG();

  Inertial.calibrate(2000);
  vex::task::sleep(2000);

  notificationHUD("PRE: DONE");

  Controller1.rumble(".");
}

void auton1()
{
  // win

  Move(0, -20, 0);
  delay(1000);
  Move(0, 50, 0);
  delay(1000);
  Move(0, 0, 0);
}

void auton2()
{
  drive(-1500, 50);

  delay(200);

  pdTurn(-45);

  delay(200);

  cata.spinFor(1800, deg, 100, velocityUnits::pct, true);

  intake.spin(directionType::fwd, 80, velocityUnits::pct);

  delay(1000);

  intake.stop();

  delay(200);

  cata.spinFor(1800, deg, 100, velocityUnits::pct, true);
}

void auton3()
{
  virtualHeading VH1(getDir());

  drivePDNew(500, 80, VH1);
  logg("here11");
  logg(VH1.get());

  delay(500);

  turnPDNew(180, 80, VH1);
  logg("here12");
  logg(VH1.get());

  delay(500);

  drivePDNew(500, 50, VH1);
  logg("here13");
  logg(VH1.get());
}

void auton4() {}

void auton5() {}

void autonG1(void)
{
  // win
  drivePd(1000, -.5);

  delay(200);

  pdTurn(180);

  delay(200);

  drivePd(500, .5);

  delay(200);

  pdTurn(0);

  delay(200);

  drivePd(-500, .5);
}

void autonG2(void)
{
  // win

  // delay(200);

  // pdTurn(180);

  // delay(200);
}

void autonG3(void) {}

void autonSkills() {}

void auton(void)
{
  leftGroup.setStopping(hold);
  rightGroup.setStopping(hold);

  if (configuration[1] == 0)
  { // Skills
    autonSkills();
  }
  else if (configuration[1] == 1)
  {
    auton1();
  }
  else if (configuration[1] == 2)
  {
    auton2();
  }
  else if (configuration[1] == 3)
  {
    auton3();
  }
  else if (configuration[1] == 4)
  {
    auton4();
  }
  else if (configuration[1] == 5)
  {
    auton5();
  }
  else if (configuration[1] == 6)
  {
    autonG1();
  }
  else if (configuration[1] == 7)
  {
    autonG2();
  }
  else if (configuration[1] == 8)
  {
    autonG3();
  }

  notificationHUD("Auton: DONE");
  Controller1.rumble(".");
}

void RCDrive(void)
{
  Ch1 = Controller1.Axis1.position(percent) * move;
  Ch3 = Controller1.Axis3.position(percent) * move;
  Ch4 = Controller1.Axis4.position(percent) * move;
  // Ch4 = Ch4 * spin;

  // Ch1 = 0.95 * Ch1 + 0.05 * Controller1.Axis1.position(percent);
  // Ch3 = 0.95 * Ch3 + 0.05 * Controller1.Axis3.position(percent);
  // Ch4 = 0.95 * Ch4 + 0.05 * Controller1.Axis4.position(percent);

  if (Ch1 < dead_zone && Ch1 > -dead_zone)
  {
    Ch1 = 0;
  }
  if (Ch2 < dead_zone && Ch2 > -dead_zone)
  {
    Ch2 = 0;
  }
  if (Ch3 < dead_zone && Ch3 > -dead_zone)
  {
    Ch3 = 0;
  }
  if (Ch4 < dead_zone && Ch4 > -dead_zone)
  {
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

  if (move_rev)
  {
    leftGroup.spin(directionType::rev, F_Right, voltageUnits::volt);
    rightGroup.spin(directionType::rev, F_Left, voltageUnits::volt);
  }
  else
  {
    leftGroup.spin(directionType::fwd, F_Left, voltageUnits::volt);
    rightGroup.spin(directionType::fwd, F_Right, voltageUnits::volt);
  }
}

void RCDrive2(void)
{
  // hmmm
}

void tankDrive(void)
{
  // hmmm
}

void btnAP()
{
  if (cata.isDone())
  {
    cata.spinFor(1800, deg, 100, velocityUnits::pct, false);
  }
}

void btnBP() {}
void btnXP() {}
void btnYP()
{
  // used
}
void btnUpP()
{
  if (move == move_max)
    move = move_min;
  else if (move == move_min)
    move = move_max;
  else
    move = move_max;
}
void btnDownP() { move_rev = !move_rev; }
void btnLeftP()
{
  // used
}
void btnRightP()
{
  // used
}

void user(void)
{

  while (keyPressed() != btnNONE)
    ;

  leftGroup.setStopping(hold);
  rightGroup.setStopping(hold);

  roller.setStopping(hold);
  cata.setStopping(hold);

  Controller1.ButtonA.pressed(btnAP);
  Controller1.ButtonB.pressed(btnBP);
  Controller1.ButtonX.pressed(btnXP);
  Controller1.ButtonY.pressed(btnYP);
  Controller1.ButtonUp.pressed(btnUpP);
  Controller1.ButtonDown.pressed(btnDownP);
  Controller1.ButtonLeft.pressed(btnLeftP);
  Controller1.ButtonRight.pressed(btnRightP);

  while (1)
  {

    switch (getValues(AUTON_DRIVE))
    {
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

    // if (Controller1.ButtonY.pressing()) {
    //   if(yeeted)  YEET.rotateFor(180, deg, 100, velocityUnits::pct);
    //   else YEET.rotateFor(-180, deg, 100, velocityUnits::pct);

    //   yeeted = !yeeted;

    //   while (Controller1.ButtonY.pressing())
    //     ;
    // }

    // if (Controller1.ButtonY.pressing())
    // {
    //   if (yeeted)
    //     Launcher.set(true);
    //   else
    //     Launcher.set(false);

    //   yeeted = !yeeted;

    //   while (Controller1.ButtonY.pressing())
    //     ;
    // }
    if (Controller1.ButtonY.pressing())
    {
      if (Competition.isCompetitionSwitch() || Competition.isFieldControl())
        return;
      else
        auton();
    }

    if (Controller1.ButtonB.pressing())
      Launcher.set(true);
    else
      Launcher.set(false);

    if (Controller1.ButtonLeft.pressing())
    {
      cata.spin(directionType::fwd, 14, voltageUnits::volt);
      cataLast = true;
    }
    else if (Controller1.ButtonRight.pressing())
    {
      cata.spin(directionType::rev, 14, voltageUnits::volt);
      cataLast = true;
    }
    else
    {
      if (cataLast)
      {
        cata.stop();
        cataLast = false;
      }
    }

    if (Controller1.ButtonL1.pressing())
    {
      intake.spin(directionType::fwd, 100, velocityUnits::pct);
    }
    else if (Controller1.ButtonL2.pressing())
    {
      intake.spin(directionType::rev, 100, velocityUnits::pct);
    }
    else
    {
      intake.stop();
    }

    if (Controller1.ButtonR1.pressing())
    {
      roller.spin(directionType::fwd, 14, voltageUnits::volt);
    }
    else if (Controller1.ButtonR2.pressing())
    {
      roller.spin(directionType::rev, 14, voltageUnits::volt);
    }
    else
    {
      roller.stop();
    }

    // looooooooooooooooooooooooooooooooooooooop
  }
}

char buf[512];

int main()
{

  calabrate();

  pre_auton();

  Competition.autonomous(auton);
  Competition.drivercontrol(user);

  while (1)
  {
    power_usage = 0;
    power_usage += L1.power(watt);
    power_usage += L2.power(watt);
    // power_usage += L3.power(watt);

    power_usage += R1.power(watt);
    power_usage += R2.power(watt);
    // power_usage += R3.power(watt);

    if (Brain.Screen.pressing() && screen == false)
    {
      screen = true;
      Brain.Screen.clearScreen();
    }

    if (screen)
    {
      Brain.Screen.setFillColor(blue);
      Brain.Screen.drawRectangle(5, 20, 60, 30);

      Brain.Screen.setPenColor(color::white);
      Brain.Screen.setFont(prop20);
      Brain.Screen.printAt(25, 39, "Back");

      Brain.Screen.setCursor(2, 10);
      Brain.Screen.print(power_usage);
      Brain.Screen.print("     ");
      // Brain.Screen.setCursor(2, 15);
      // Brain.Screen.print(ArmRight.position(deg));
      // Brain.Screen.print("  ");

      Brain.Screen.setCursor(7, 25);
      Brain.Screen.print(cata.position(deg));
      Brain.Screen.print("  ");

      Brain.Screen.setCursor(2, 25);
      Brain.Screen.print(getDir());
      Brain.Screen.print("  ");
      Brain.Screen.setCursor(2, 30);
      Brain.Screen.print(Inertial.rotation());
      Brain.Screen.print("  ");

      Brain.Screen.setCursor(4, 10);
      Brain.Screen.print(cataROT);
      Brain.Screen.print("  ");
      Brain.Screen.setCursor(4, 15);
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

      if (Brain.Screen.pressing())
      {

        int xPos = Brain.Screen.xPosition();
        int yPos = Brain.Screen.yPosition();
        while (Brain.Screen.pressing())
          ;

        if (xPos > 5 && xPos < 5 + 60 && yPos > 20 && yPos < 20 + 30)
        {
          screen = false;
          Brain.Screen.clearScreen();
        }
      }
    }
    else
    {
      if (getValues(AUTON_COLOR) == RED)
      {
        Brain.Screen.drawImageFromFile("red.png", 0, 0);
        // Brain.Screen.setFillColor(red);
      }
      else if (getValues(AUTON_COLOR) == BLUE)
      {
        Brain.Screen.drawImageFromFile("blue.png", 0, 0);
        // Brain.Screen.setFillColor(blue);
      }
      else if (getValues(AUTON_COLOR) == 2)
      {
        Brain.Screen.drawImageFromFile("gay.png", 0, 0);
      }
    }

    if (tempStatus != currStatus())
    { //  || autoTempStatus != automatic
      statusHUD();
      tempStatus = currStatus();
      // autoTempStatus = automatic;
    }
    delay(200);
  }
}
