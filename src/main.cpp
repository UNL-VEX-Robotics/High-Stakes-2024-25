/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       closm                                                     */
/*    Created:      9/11/2024, 10:36:44 AM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "odom.h"
#include "grapher.h"
#include "drivetrain.h"
#include "auton-selector.h"
#include "intake.h"
#include "pnuematics.h"
 
using namespace vex;

// A global instance of competition
competition Competition;

/* ---------- Devices ---------- */
vex::brain Brain;
controller Controller1 = controller(primary);

motor RightFront = motor(PORT1, ratio6_1, false);
motor RightMiddle = motor(PORT2, ratio6_1, false);
motor RightTop = motor(PORT3, ratio6_1, true);
motor RightBack = motor(PORT6, ratio6_1, false);

motor LeftFront = motor(PORT5, ratio6_1, true);
motor LeftMiddle = motor(PORT10, ratio6_1, true);
motor LeftTop = motor(PORT9, ratio6_1, false);
motor LeftBack = motor(PORT19, ratio6_1, true);

motor LeftIntake = motor(PORT7, ratio18_1, false);
motor RightIntake = motor(PORT14, ratio18_1, true);
motor_group IntakeGroup = motor_group(LeftIntake, RightIntake);

motor LeftLift = motor(PORT8, ratio36_1, true);
motor RightLift = motor(PORT12, ratio36_1, false);
motor_group LiftGroup = motor_group(LeftLift, RightLift);

motor_group Right = motor_group(RightFront, RightMiddle, RightTop, RightBack);
motor_group Left = motor_group(LeftFront, LeftMiddle, LeftTop, LeftBack);

inertial Inertial = inertial(PORT4);
optical Optical = optical(PORT15);
distance Distance = distance(PORT13);
potV2 LiftPotentiometer = potV2(Brain.ThreeWirePort.G);
line RightLineFollower = line(Brain.ThreeWirePort.D);
line LeftLineFollower = line(Brain.ThreeWirePort.A);


led MogoClamp = led(Brain.ThreeWirePort.H);
led mogoRush = led(Brain.ThreeWirePort.F);
led mogoRushLeft = led(Brain.ThreeWirePort.A);


/* ---------- Tasks ---------- */
vex::task dt_drivetrain;
vex::task dt_intake;
vex::task dt_intake_control;
vex::task dt_ladybrown;
vex::task dt_ladybrown_control;
vex::task dt_pnuematic_control;

vex::task at_intake;
vex::task at_ladybrown;

vex::task gt_odometry;

/* ---------- Global Problem Solvers ---------- */

/**
 * Leo's solution for passing member functions to vex::task
 * to use: launch_task(std::bind(&class::func, &classInstance, funcParams...))
 * Don't ask questions lol
 */
template <class F>
vex::task launch_task(F&& function) {
  //static_assert(std::is_invocable_r_v<void, F>);
  return vex::task([](void* parameters) {
    std::unique_ptr<std::function<void()>> ptr{static_cast<std::function<void()>*>(parameters)};
    (*ptr)();
    return 0;
  }, new std::function<void()>(std::forward<F>(function)));
}

/* ---------- Objects ---------- */
Graph graph = Graph(&Brain.Screen);
ladybrown Ladybrown = ladybrown(&LiftGroup, -200, -90, 0, 160);
intake Intake = intake(&IntakeGroup, &Optical, 140, &Ladybrown, 2000 / 3);

odometry Odom = odometry(odometry::odometry_pod(odometry::odometry_pod::VERTICAL, &LeftFront, 5.65625, 0.0212712), odometry::odometry_pod(), &Inertial);
chassis Drivetrain = chassis(std::bind(&odometry::getPosition, &Odom), &Left, &Right, &Inertial, 11.3125, 0.0212712);

/* ---------- Global Variables ---------- */
enum Autonomous
{
  NONE,
  RED,
  BLUE,
  SKILLS,
  BLUE_SAFE, 
  RED_SAFE
};

bool isRed = false;
Autonomous auton = NONE;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
void pre_auton(void) 
{

  IntakeGroup.setPosition(0, degrees);
  Intake.setBrakeType(brake);
  Intake.setColor(true);

  //PIDs
  Drivetrain.setDriveConstants(0.70, 0.02, 2.5, 9, 0.25, 30, -12, 12, 0.2);
  Drivetrain.setTurnConstants(0.15, 0.01, 1.1, 15, 0.25, 30, -12, 12);
  Drivetrain.setSwingConstants(0.2, 0.01, 0.55, 15, 0.5, 30, -12, 12);

  Drivetrain.setArcConstants(0.25, 0.01, 0.7, 15, 0.5, 50, -12, 12);

  Ladybrown.setPIDConstants(0.5, 0, 0, 0, 20);
  Ladybrown.setCurrentPosition(-200);
  LiftGroup.setPosition(-200, deg);
  LeftLift.setPosition(-200, deg);
  RightLift.setPosition(-200, deg);

  if (auton == NONE) 
  {
    // run auton selector

    //colors
    color redSelected = color(125, 0, 0);
    color blueSelected = color(0, 0, 125);
    color skillsColor = color(235, 255, 0);
    color skillsSelectedColor = color(115, 125, 0);

    //buttons
    button redButton = button(&Brain.Screen, 25, 25, 200, 80, red, red, 1, white, "Red");
    button redSafeButton = button(&Brain.Screen, 0, 25, 25, 190, red, red, 1, white, "");
    button blueButton = button(&Brain.Screen, 255, 25, 200, 80, blue, blue, 1, white, "Blue");
    button blueSafeButton = button(&Brain.Screen, 455, 25, 25, 190, blue, blue, 1, white, "");
    button skillsButton = button(&Brain.Screen, 25, 135, 200, 80, skillsColor, skillsColor, 1, black, "Skills");
    button calibrateButton = button(&Brain.Screen, 255, 135, 200, 80, white, white, 1, black, "Calibrate");

    while (true)
    {
      // draw buttons
      redButton.draw();
      redSafeButton.draw();
      blueButton.draw();
      blueSafeButton.draw();
      skillsButton.draw();
      calibrateButton.draw();

      waitUntil(Brain.Screen.pressing());

      // pressing logic

      if (calibrateButton.isPressing()) break;

      if (redButton.isPressing())
      {
        if (auton == RED) auton = NONE;
        else auton = RED;
      }

      if (redSafeButton.isPressing())
      {
        if (auton == RED_SAFE) auton = NONE;
        else auton = RED_SAFE;
      }

      if (blueButton.isPressing())
      {
        if (auton == BLUE) auton = NONE;
        else auton = BLUE;
      }

      if (blueSafeButton.isPressing())
      {
        if (auton == BLUE_SAFE) auton = NONE;
        else auton = BLUE_SAFE;
      }

      if (skillsButton.isPressing())
      {
        if (auton == SKILLS) auton = NONE;
        else auton = SKILLS;
      }

      waitUntil(!Brain.Screen.pressing());

      // update color

      switch (auton)
      {
        case NONE:
          redButton.changeColor(red, red, white);
          blueButton.changeColor(blue, blue, white);
          skillsButton.changeColor(skillsColor, skillsColor, black);
          redSafeButton.changeColor(red, red, white);
          blueSafeButton.changeColor(blue, blue, white);
          break;
        case RED:
          redButton.changeColor(redSelected, redSelected, white);
          blueButton.changeColor(blue, blue, white);
          skillsButton.changeColor(skillsColor, skillsColor, black);
          redSafeButton.changeColor(red, red, white);
          blueSafeButton.changeColor(blue, blue, white);
          break;
        case BLUE:
          redButton.changeColor(red, red, white);
          blueButton.changeColor(blueSelected, blueSelected, white);
          skillsButton.changeColor(skillsColor, skillsColor, black);
          redSafeButton.changeColor(red, red, white);
          blueSafeButton.changeColor(blue, blue, white);
          break;
        case SKILLS:
          redButton.changeColor(red, red, white);
          blueButton.changeColor(blue, blue, white);
          skillsButton.changeColor(skillsSelectedColor, skillsSelectedColor, white);
          redSafeButton.changeColor(red, red, white);
          blueSafeButton.changeColor(blue, blue, white);
          break;
        case RED_SAFE:
          redButton.changeColor(red, red, white);
          blueButton.changeColor(blue, blue, white);
          skillsButton.changeColor(skillsColor, skillsColor, black);
          redSafeButton.changeColor(redSelected, redSelected, white);
          blueSafeButton.changeColor(blue, blue, white);
          break;
        case BLUE_SAFE:
          redButton.changeColor(red, red, white);
          blueButton.changeColor(blue, blue, white);
          skillsButton.changeColor(skillsColor, skillsColor, black);
          redSafeButton.changeColor(red, red, white);
          blueSafeButton.changeColor(blueSelected, blueSelected, white);
          break;
      }
    }
  }

  Brain.Screen.clearScreen(red);
  task::sleep(1000);

  //  Calibration
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.setPenColor(white);
  Brain.Screen.setFillColor(black);
  Brain.Screen.print("Calibrating...");

  Inertial.startCalibration();
  do {
    task::sleep(50);
  } while (Inertial.isCalibrating());

  //  Update display for clarification
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.setPenColor(green);
  Brain.Screen.print("Calibrated.");
  task::sleep(10);

  //  check if devices are connected
  Brain.Screen.setPenColor(red);
  Brain.Screen.newLine();
  
  if(!LeftFront.installed()) Brain.Screen.print("Left Front Drive Motor Disconnected! \n");
  if(!LeftMiddle.installed()) Brain.Screen.print("Left Middle Drive Motor Disconnected! \n");
  if(!LeftTop.installed()) Brain.Screen.print("Left Top Drive Motor Disconnected! \n");
  if(!LeftBack.installed()) Brain.Screen.print("Left Back Drive Motor Disconnected! \n");
  
  if(!RightFront.installed()) Brain.Screen.print("Right Front Drive Motor Disconnected! \n");
  if(!RightMiddle.installed()) Brain.Screen.print("Right Middle Drive Motor Disconnected! \n");
  if(!RightTop.installed()) Brain.Screen.print("Right Top Drive Motor Disconnected! \n");
  if(!RightBack.installed()) Brain.Screen.print("Right Back Drive Motor Disconnected! \n");

  if(!LeftIntake.installed()) Brain.Screen.print("Left Intake Motor Disconnected! \n");
  if(!LeftLift.installed()) Brain.Screen.print("Left Ladybrown Motor Disconnected! \n");

  if(!Inertial.installed()) Brain.Screen.print("Inertial Sensor Disconnected! \n");
  if(!Optical.installed()) Brain.Screen.print("Optical Sensor Disconnected \n");
}

/* ---------- Autonomous Functions ---------- */

/**
 * @brief sets the position of the robot and starts the odometry task
 * 
 * @param initialX the x-coordinate of the robot
 * @param initialY the y-coordinate of the robot
 * @param initialHeading the heading of the robot
 */
void startOdometry(float initialX, float initialY, float initialHeading)
{
  gt_odometry = launch_task(std::bind(&odometry::startTracking, &Odom, initialX, initialY, initialHeading));
}

/**
 * @brief stops intake when ring in ready to be scored
 */
int holdRing(uint8_t stage)
{
  if(stage == 1) waitUntil(Distance.objectDistance(inches) < 1.0);
  else waitUntil(Optical.isNearObject());
  Intake.setSpeed(0);

  return 0;
}

/**
 * @brief Activates the mogo clamp after a delay
 * 
 * @param mS millisecond delay
 */
int activateMogoClamp(int mS)
{
  task::sleep(mS);
  MogoClamp.off();

  return 0;
}

bool ringStored = false;
int store_ladybrown()
{
  ringStored = false;
  Intake.setSpeed(100);
  for (int i = 0; i < 2000; i += 10)
  {
    if (Optical.isNearObject()) break;
    else task::sleep(10);
  }
  ringStored = true;
  task::sleep(400);
  Intake.setSpeed(-100);
  task::sleep(400);
  Intake.setSpeed(0);

  return 0;
}

void skills_start()
{
  task hold_ring_task;

  Inertial.setHeading(90, deg);
  Ladybrown.setTarget(ladybrown::READY);

  // first ring onto alliance stake
  Intake.setSpeed(100);
  hold_ring_task = launch_task(std::bind(holdRing, 2));
  Drivetrain.driveFor(7.5);
  Drivetrain.turnTo(273);
  hold_ring_task.stop();

  task storeLadybrown = task(store_ladybrown);

  Drivetrain.driveFor(12);
  Drivetrain.setDriveSpeed(4, volt);
  task::sleep(500);
  Drivetrain.driveFor(-8.25);
  waitUntil(ringStored);

  Ladybrown.setTarget(ladybrown::SCORE);
  task::sleep(650);
  Ladybrown.setTarget(ladybrown::DOWN);
}

/**
 * @brief skills route
 */
void skills()
{
  task hold_ring_task;

  float initialTime = Brain.Timer.systemHighResolution();

  skills_start();
  task::sleep(50);

  // ring at (-24, 24)
  Drivetrain.turnTo(53);
  Intake.setSpeed(100);
  hold_ring_task = launch_task(std::bind(holdRing, 2));
  Drivetrain.driveFor(42);
  task::sleep(250);

  // mobile goal at (-24, 48)
  Drivetrain.turnTo(180);
  Drivetrain.driveFor(-14);
  task activateMogo = launch_task(std::bind(activateMogoClamp, 350));
  Drivetrain.driveFor(-7);
  MogoClamp.off();
  task::sleep(200);

  // ring at (0, 48) AND (24, 48)
  Drivetrain.turnTo(90);
  hold_ring_task.stop();
  Intake.setSpeed(100);
  Drivetrain.driveFor(39);
  double currentDistance = Left.position(deg) * 0.0212712;
  Drivetrain.setDriveSpeed(4, volt);
  while (Left.position(deg) * 0.0212712 < currentDistance + 17) task::sleep(5);
  Drivetrain.stopDrive();
  task::sleep(1000);
  Drivetrain.driveFor(12);
  Drivetrain.driveFor(-12);
  Intake.setSpeed(100);
  Drivetrain.turnTo(270);
  Intake.setSpeed(100);

  // ring at (-48, 48)
  Drivetrain.driveFor(90);
  Intake.setSpeed(100);

  // ring in corner
  Drivetrain.turnTo(340);
  Drivetrain.driveFor(24, 1);

  // mobile goal in corner
  Drivetrain.driveFor(-15);
  Drivetrain.turnTo(155);
  Drivetrain.driveFor(-24, 1);
  Intake.setSpeed(-100);
  task::sleep(1000);
  MogoClamp.on();

  // wall reset
  Drivetrain.driveFor(24);
  Drivetrain.turnTo(90);
  Drivetrain.setDriveSpeed(-5, volt);
  task::sleep(1000);
  Inertial.setHeading(90, deg);
  Drivetrain.driveFor(24);
  Drivetrain.turnTo(180);
  Drivetrain.driveFor(-24, 1.5);
  Drivetrain.setDriveSpeed(-5, volt);
  task::sleep(750);

  //ring at (0, 60)
  Drivetrain.swingFor(left, 90);
  Intake.setSpeed(100);
  hold_ring_task = launch_task(std::bind(holdRing, 2));
  Drivetrain.driveFor(60);
  
  // mobile goal at (24, 24)
  Drivetrain.turnTo(0);
  Drivetrain.driveFor(-18);
  activateMogo = launch_task(std::bind(activateMogoClamp, 750));
  Drivetrain.driveFor(-12);
  Drivetrain.driveFor(-5);
  hold_ring_task.stop();

  // ring at (48, 24)
  Drivetrain.turnTo(90);
  Intake.setSpeed(100);
  task::sleep(500);
  Drivetrain.driveFor(30);

  //corner
  Drivetrain.turnTo(25);
  Drivetrain.driveFor(30);
  task::sleep(500);
  Drivetrain.driveFor(24, 1);
  task::sleep(250);
  Drivetrain.driveFor(-12);
  task::sleep(250);
  Drivetrain.driveFor(24, 1);
  task::sleep(250);
  Drivetrain.driveFor(-12);
  Drivetrain.turnTo(200);
  Drivetrain.driveFor(-24, 1);
  Intake.setSpeed(-100);
  task::sleep(1000);
  MogoClamp.on();
  Drivetrain.driveFor(12);

  // end
  Intake.setBrakeType(coast);
  Intake.setSpeed(0);
  Drivetrain.stopDrive(coast);
  Brain.Screen.clearScreen(purple);
  Controller1.Screen.print((Brain.Timer.systemHighResolution() - initialTime) * 0.000001);
}

void purdueAuto(bool isRed)
{
  int mult = (isRed ? 1 : -1);
  //Intake.setColorSortOffset(0);
  // set up
  task hold_ring_task;
  float initialTime = Brain.Timer.systemHighResolution();
  Inertial.setHeading((isRed ? 90 : 270), deg);
  Ladybrown.setTarget(ladybrown::DOWN);

  // rush mogo
  if (isRed) mogoRush.off();
  else mogoRushLeft.off();
  Drivetrain.driveFor(37.5);
  if (isRed) mogoRush.on();
  else mogoRushLeft.on();
  task::sleep(250);
  Drivetrain.turnTo(180);
  if (isRed) mogoRush.off();
  else mogoRushLeft.off();
  Drivetrain.driveFor(24);
  if (isRed) mogoRush.on();
  else mogoRushLeft.on();

  Drivetrain.driveFor(-6);
  Drivetrain.turnTo(0);
  Drivetrain.driveFor(-8);
  Drivetrain.driveFor(-6);
  MogoClamp.off();
  task::sleep(250);

  // ring at (-48, 48)
  Drivetrain.swingFor((isRed ? left : right), 60);
  Intake.setSpeed(100);
  Drivetrain.driveFor(12);
  double currentDistance = Left.position(deg) * 0.0212712;
  Drivetrain.setDriveSpeed(3.5, volt);
  while (Left.position(deg) * 0.0212712 < currentDistance + 30) task::sleep(5);

  // drop goal
  Drivetrain.driveFor(-46);
  MogoClamp.on();
  
  // goal at (-48, 0)
  hold_ring_task = launch_task(std::bind(holdRing, 2));
  Drivetrain.turnTo(270 * mult);
  Drivetrain.driveFor(28);
  Drivetrain.turnTo(90 * mult);
  Drivetrain.setDriveSpeed(-4, volt);
  task::sleep(1000);
  Inertial.setHeading((isRed ? 90 : 270), deg);
  Drivetrain.driveFor(14);
  Drivetrain.turnTo(0);
  
  Drivetrain.driveFor(-24);
  Drivetrain.driveFor(-8);
  MogoClamp.off();
  task::sleep(500);
  Drivetrain.turnTo(0);
  
  //ring in front of alliance stake
  Drivetrain.driveFor(24);
  Drivetrain.swingFor((isRed ? left : right), 180);
  Drivetrain.turnTo(178 * mult);
  hold_ring_task.stop();
  Intake.setSpeed(100);
  Drivetrain.driveFor(34);
  task::sleep(250);
  
  // post
  Drivetrain.turnTo(90 * mult);
  currentDistance = Left.position(deg) * 0.0212712;
  Drivetrain.setDriveSpeed(3.5, volt);
  int t = 0;
  while (Left.position(deg) * 0.0212712 < currentDistance + 30 && t < 3000) 
  {
    task::sleep(5);
    t += 5;
  }
  Drivetrain.stopDrive(coast);
  Ladybrown.setTarget(Ladybrown.SCORE);

  // end
  Intake.setBrakeType(coast);
  //Intake.setSpeed(0);
  Drivetrain.stopDrive(coast);
  Brain.Screen.clearScreen(purple);
  Controller1.Screen.print((Brain.Timer.systemHighResolution() - initialTime) * 0.000001);
}

void safeAuto(bool isRed)
{
  // setup
  Inertial.setHeading((isRed ? 210 : 180 - 30), deg);
  int mult = (isRed ? 1 : -1);
  task hold_ring_task;
  float initialTime = Brain.Timer.systemHighResolution();

  // alliance stake
  Ladybrown.setTarget(Ladybrown.SCORE);
  task::sleep(400);
  Ladybrown.setTarget(Ladybrown.DOWN);
  task::sleep(100);

  // ring at (-60, 0)
  Drivetrain.swingFor((isRed ? right : left), -30);
  Intake.setSpeed(100);
  hold_ring_task = launch_task(std::bind(holdRing, 2));
  Drivetrain.driveFor(24);

  // mobile goal at (-48, 0)
  Drivetrain.turnTo(240 * mult);
  Drivetrain.driveFor(-12);
  Drivetrain.driveFor(-6);
  MogoClamp.off();
  task::sleep(500);

  // ring at (-48. 48)
  Drivetrain.turnTo(345 * mult);
  Drivetrain.turnTo(345 * mult);
  hold_ring_task.stop();
  Intake.setSpeed(100);
  Drivetrain.driveFor(56);

  //touch post
  Drivetrain.turnTo(147 * mult);
  Drivetrain.setDriveSpeed(4, volt);
  task::sleep(3000);

  // end
  Intake.setBrakeType(coast);
  Intake.setSpeed(0);
  Drivetrain.stopDrive(coast);
  Brain.Screen.clearScreen(purple);
  Controller1.Screen.print((Brain.Timer.systemHighResolution() - initialTime) * 0.000001);
}

vex::color getPrintColor()
{
  switch (auton)
    {
      case NONE: return white;
      case RED: 
      case RED_SAFE:
        return red;
      case BLUE: 
      case BLUE_SAFE:
        return BLUE;
      case SKILLS: return vex::color(235, 255, 0);
    }
}

void autonomous(void) {
  at_intake = launch_task(std::bind(&intake::intake_task, &Intake));
  at_ladybrown = launch_task(std::bind(&ladybrown::ladybrown_task, &Ladybrown));
  Intake.setColor(!(auton == BLUE || auton == BLUE_SAFE));
  Intake.setColorSort(true);    

  Brain.Screen.clearScreen(getPrintColor());

  switch (auton)
  {
    case RED:
      purdueAuto(true);

      break;
    case BLUE:
      purdueAuto(true);
      
      break;
    case RED_SAFE:
      safeAuto(true);

      break;
    case BLUE_SAFE:
      safeAuto(false);

      break;
    case SKILLS:
      skills();

      break;
    case NONE:
      break;
  }
}

/* ---------- User Control Functions ---------- */

/**
 * @brief definition for the task that controls the drivetrain
 */
int drivetrain_task()
{
  while(true)
  {
    Left.spin(forward, (Controller1.Axis3.position(percent) + Controller1.Axis1.position(percent)) * 0.12, volt);
    Right.spin(forward, (Controller1.Axis3.position(percent) - Controller1.Axis1.position(percent)) * 0.12, volt);

    task::sleep(10);
  }
}

int ladybrownTarget = 1; // down
/**
 * @brief definition for the task that controls the intake
 */
int intake_control_task()
{
  bool L1_wasPressing = false;
  bool L2_wasPressing = false;
  bool Down_wasPressing = false;

  bool intakeOn = false;
  bool spinForward = true;
  bool ladybrownMacro = false;

  bool enableColorSort = !(auton == SKILLS);
  Intake.setColor(!(auton == BLUE || auton == BLUE_SAFE));
  Intake.setColorSort(enableColorSort);
  Intake.setBrakeType(brake);

  int mS_passed = 0;
  while(true)
  {
    if(Controller1.ButtonL1.pressing() && !L1_wasPressing) intakeOn = !intakeOn;
    if(Controller1.ButtonL2.pressing() && !L2_wasPressing) spinForward = !spinForward;
    if(Controller1.ButtonDown.pressing() && !Down_wasPressing) 
    { 
      enableColorSort = !enableColorSort;
      Intake.setColorSort(enableColorSort);
    }

    if(ladybrownTarget == 2 && Optical.isNearObject())
    {
      task::sleep(400);
      Intake.setSpeed(-100);
      task::sleep(400);
      Intake.setSpeed(0);
      ladybrownMacro = false;
      intakeOn = false;
    }

    if(ladybrownMacro && mS_passed % 250 == 0) Controller1.rumble(".");

    if(intakeOn && spinForward) Intake.setSpeed(100);
    else if(intakeOn) Intake.setSpeed(-100);
    else Intake.setSpeed(0);

    L1_wasPressing = Controller1.ButtonL1.pressing();
    L2_wasPressing = Controller1.ButtonL2.pressing();
    Down_wasPressing = Controller1.ButtonDown.pressing();

    // Brain.Screen.clearScreen();
    // Brain.Screen.setCursor(1, 1);
    // Brain.Screen.print((1920 / 3 )- ((int)IntakeGroup.position(deg) % (1920 / 3)));
    task::sleep(10);
    mS_passed += 10;
  }
}


/**
 * @brief definition for the task that controls the lady brown mechanism
 */
int ladybrown_control_task()
{
  
  bool R1_wasPressing = false;
  bool R2_wasPressing = false;

  Ladybrown.setTarget(ladybrown::DOWN);
  

  while(true)
  {
    if(Controller1.ButtonR1.pressing() && !R1_wasPressing && ladybrownTarget < 3) 
    {
      ladybrownTarget++;

      switch (ladybrownTarget)
      {
      case 1:
        Ladybrown.setTarget(ladybrown::DOWN);
        break;
      case 2:
        Ladybrown.setTarget(ladybrown::READY);
        break;
      case 3:
        Ladybrown.setTarget(ladybrown::SCORE);
        break;    
      }

      Controller1.rumble(".");
    }
    R1_wasPressing = Controller1.ButtonR1.pressing();
    if(Controller1.ButtonR2.pressing() && !R2_wasPressing && ladybrownTarget > 1) 
    {
      ladybrownTarget--;

      switch (ladybrownTarget)
      {
      case 1:
        Ladybrown.setTarget(ladybrown::DOWN);
        Controller1.Screen.clearLine();
        Controller1.Screen.print("DOWN");
        break;
      case 2:
        Ladybrown.setTarget(ladybrown::READY);
        Controller1.Screen.clearLine();
        Controller1.Screen.print("READY");
        break;
      case 3:
        Ladybrown.setTarget(ladybrown::SCORE);
        Controller1.Screen.clearLine();
        Controller1.Screen.print("SCORE");
        break; 
      }

      Controller1.rumble(".");
    }
    R2_wasPressing = Controller1.ButtonR2.pressing();

    task::sleep(10);
  }
}


/**
 * @brief controls the pistons on the robot
 */
int pnuematic_control_task()
{
  bool X_wasPressing = false;
  bool toggleMogoMech = false;
  bool wasEnabled = false;
  while(true)
  {
    if(Controller1.ButtonX.pressing() && !X_wasPressing)
    {
      toggleMogoMech = !toggleMogoMech;

      if(toggleMogoMech) MogoClamp.off();
      else MogoClamp.on();
    }
    X_wasPressing = Controller1.ButtonX.pressing();
    
    wasEnabled = Competition.isDriverControl() && Competition.isEnabled();

    task::sleep(10);
  }
}

void usercontrol(void) 
{
  dt_intake = launch_task(std::bind(&intake::intake_task, &Intake));
  dt_ladybrown = launch_task(std::bind(&ladybrown::ladybrown_task, &Ladybrown));

  Inertial.setHeading(90, deg);
  Ladybrown.setTarget(Ladybrown.DOWN);

  // first ring onto alliance stake
  if (auton == SKILLS)
  {
    skills_start();
  }

  dt_intake_control = task(intake_control_task);
  dt_ladybrown_control = task(ladybrown_control_task);
  dt_pnuematic_control = task(pnuematic_control_task);
  dt_drivetrain = task(drivetrain_task);

  while (1) 
  {
    task::sleep(10);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}