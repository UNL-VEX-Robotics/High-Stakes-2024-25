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


led MogoClamp = led(Brain.ThreeWirePort.H);


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

bool isRed = false;

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
intake Intake = intake(&IntakeGroup, &Optical, 1920 / 3, &Ladybrown, 2000 / 3);

odometry Odom = odometry(odometry::odometry_pod(odometry::odometry_pod::VERTICAL, &LeftFront, 5.65625, 0.0212712), odometry::odometry_pod(), &Inertial);
chassis Drivetrain = chassis(std::bind(&odometry::getPosition, &Odom), &Left, &Right, &Inertial, 11.3125, 0.0212712);

/* ---------- Global Variables ---------- */

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
  //  Calibration
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.setPenColor(white);
  Brain.Screen.setFillColor(black);
  Brain.Screen.print("Calibrating...");

  IntakeGroup.setPosition(0, degrees);
  Intake.setBrakeType(brake);
  Intake.setColor(true);

  Inertial.startCalibration();
  do {
    task::sleep(50);
  } while (Inertial.isCalibrating());

  Ladybrown.setCurrentPosition(-200);

  //PIDs
  Drivetrain.setDriveConstants(0.75, 0.005, 1, 9, 0.75, 30, -12, 12, 0.5);
  Drivetrain.setTurnConstants(0.15, 0.01, 0.6, 15, 0.5, 50, -12, 12);
  Drivetrain.setSwingConstants(0.2, 0.005, 0.3, 22, 0.5, 50, -12, 12);
  Drivetrain.setArcConstants(0.25, 0.01, 0.7, 15, 0.5, 50, -12, 12);

  Ladybrown.setPIDConstants(0.5, 0, 0, 0, 20);

  Intake.setColorSort(false);


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

/**
 * @brief skills route
 */
void skills()
{
  float initialTime = Brain.Timer.systemHighResolution();
  Inertial.setHeading(90, deg);


  Ladybrown.setTarget(Ladybrown.DOWN);



  // first ring onto alliance stake
  Intake.setSpeed(100);
  task hold_ring_task = launch_task(std::bind(holdRing, 2));
  Drivetrain.driveFor(14);
  Drivetrain.driveFor(-14.75, 3);
  hold_ring_task.stop();
  holdRing(2);
  Intake.setSpeed(100);
  task::sleep(400);

  // ring 1 and corner ring
  Intake.setSpeed(-100);
  Drivetrain.swingTo(left, 10, 2);
  Intake.setSpeed(100);
  hold_ring_task = launch_task(std::bind(holdRing, 1));
  Drivetrain.driveFor(50);
  task::sleep(500);
  Drivetrain.arcFor(left, 7, 100);
  hold_ring_task.stop();
  Intake.setSpeed(100);
  hold_ring_task = launch_task(std::bind(holdRing, 2));
  Drivetrain.driveFor(16, 1.5);

  //pick up mogo and score
  Drivetrain.driveFor(-15);
  Drivetrain.swingTo(left, 310, 2);
  Drivetrain.driveFor(-18);
  Drivetrain.setDriveSpeed(-6, volt);
  activateMogoClamp(300);
  Intake.setSpeed(100);
  task::sleep(50);
  Drivetrain.stopDrive();
  task::sleep(1000);

  // corner
  Drivetrain.turnTo(182, 1.5);
  Drivetrain.driveFor(28);
  Drivetrain.turnTo(150, 1.5);
  task::sleep(250);
  Drivetrain.driveFor(-56);
  MogoClamp.on();
  
  hold_ring_task.stop();
  Intake.setSpeed(0);

  Drivetrain.swingTo(left, 90);
  //hold_ring_task = launch_task(std::bind(holdRing, 2));
  
  // pick up ring and next mogo
  Drivetrain.driveFor(50);
  task::sleep(250);
  Intake.setSpeed(100);
  hold_ring_task = launch_task(std::bind(holdRing, 1));
  Drivetrain.driveFor(6);
  task::sleep(500);
  

  Drivetrain.turnTo(180, 1.5);
  hold_ring_task.stop();
  hold_ring_task = launch_task(std::bind(holdRing, 2));
  Intake.setSpeed(100);
  task::sleep(500);
  Drivetrain.driveFor(24);
  task::sleep(750);

  hold_ring_task.stop();
  Drivetrain.driveFor(-16);
  Drivetrain.swingTo(left, 225);

  Drivetrain.driveFor(5);
  Drivetrain.turnTo(320, 1.5);
  Drivetrain.driveFor(-30);
  task::sleep(250);
  Drivetrain.driveFor(-6);

  activateMogoClamp(300);
  Intake.setSpeed(100);
  Drivetrain.driveFor(-6);
  Drivetrain.turnTo(0, 1.5);
  Drivetrain.driveFor(26);
  Drivetrain.driveFor(-3);
  Drivetrain.turnTo(90, 1.5);
  Drivetrain.driveFor(26);
  Drivetrain.driveFor(-3);
  Drivetrain.turnTo(180, 1.5);
  Drivetrain.driveFor(26);
  Drivetrain.driveFor(-26);
  Drivetrain.turnTo(45, 1.5);
  Intake.setSpeed(100);
  Drivetrain.driveFor(24);
  task::sleep(250);
  Drivetrain.driveFor(-15);
  Drivetrain.turnTo(180, 1.5);
  Intake.setSpeed(-100);
  Drivetrain.turnTo(45, 1.5);
  Intake.setSpeed(100);
  Drivetrain.driveFor(15);
  task::sleep(250);
  Drivetrain.driveFor(-15);
  Drivetrain.turnTo(225, 1.5);
 



  Drivetrain.driveFor(-15);
  MogoClamp.on();



  Brain.Screen.clearScreen(purple);
  Controller1.Screen.print((Brain.Timer.systemHighResolution() - initialTime) * 0.000001);
}

void centerMogo()
{

  int microsecondsStart = Brain.Timer.systemHighResolution();
  if(isRed) Inertial.setHeading(105, deg);
  else Inertial.setHeading(255, deg);

  int mult = (isRed) ? 1 : -1;
  Ladybrown.setTarget(Ladybrown.DOWN);
  Intake.setSpeed(100);
  task stopIntake = launch_task(std::bind(holdRing, 2));

  Drivetrain.driveFor(43.5);
  Drivetrain.turnTo(0 * mult);
  waitUntil((int)IntakeGroup.position(deg) % 1920 > 400 || IntakeGroup.velocity(pct) < 5);
  Intake.setSpeed(0);
  Drivetrain.driveFor(-5);

  if(isRed) Drivetrain.swingFor(right, -60, 1.5);
  else Drivetrain.swingFor(left, -60, 1.5);

  Drivetrain.driveFor(3, 1.5);

  Drivetrain.driveFor(-6);
  MogoClamp.off();
  task::sleep(250);

  stopIntake.stop();
  Intake.setSpeed(100);
  task::sleep(5000);
  if(isRed)Drivetrain.swingFor(right, 35, 1.5);
  else Drivetrain.swingFor(left, 35, 1.5);

  Drivetrain.driveFor(32);
  Drivetrain.driveFor(8);
  Drivetrain.driveFor(-8);

  Drivetrain.turnFor(180 * mult);
  Drivetrain.driveFor(34);
  Drivetrain.turnTo(210 * mult);

  Drivetrain.driveFor(36);
  Drivetrain.driveFor(-6);
  Drivetrain.turnTo(180 * mult);
  Drivetrain.driveFor(26);
  task::sleep(500);

  Drivetrain.turnTo(343 * mult);
  Drivetrain.driveFor(100);
  Drivetrain.turnTo(135 * mult);
  MogoClamp.on();
  Drivetrain.setDriveSpeed(-4, volt);

  task::sleep(2000);

  Drivetrain.driveFor(12);
  Drivetrain.turnTo(270 * mult);
  Drivetrain.driveFor(-40);


  Controller1.Screen.print((float)(Brain.Timer.systemHighResolution() - microsecondsStart) * 0.000001);
}

void autonomous(void) {
  at_intake = launch_task(std::bind(&intake::intake_task, &Intake));
  at_ladybrown = launch_task(std::bind(&ladybrown::ladybrown_task, &Ladybrown));

  centerMogo();


  //skills();
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

  Intake.setColorSort(false);
  Intake.setBrakeType(brake);
  while(true)
  {
    if(Controller1.ButtonL1.pressing() && !L1_wasPressing) intakeOn = !intakeOn;
    if(Controller1.ButtonL2.pressing() && !L2_wasPressing) spinForward = !spinForward;
    if(Controller1.ButtonDown.pressing() && !Down_wasPressing) ladybrownMacro = !ladybrownMacro;

    if(ladybrownMacro && Optical.isNearObject())
    {
      task::sleep(400);
      Intake.setSpeed(-100);
      task::sleep(400);
      Intake.setSpeed(0);
      ladybrownMacro = false;
      intakeOn = false;
    }

    if(intakeOn && spinForward) Intake.setSpeed(100);
    else if(intakeOn) Intake.setSpeed(-100);
    else Intake.setSpeed(0);

    L1_wasPressing = Controller1.ButtonL1.pressing();
    L2_wasPressing = Controller1.ButtonL2.pressing();
    Down_wasPressing = Controller1.ButtonDown.pressing();

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print((1920 / 3 )- ((int)IntakeGroup.position(deg) % (1920 / 3)));
    task::sleep(10);
  }
}

/**
 * @brief definition for the task that controls the lady brown mechanism
 */
int ladybrown_control_task()
{
  
  bool R1_wasPressing = false;
  bool R2_wasPressing = false;
  int ladybrownTarget = 1; // down

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
  dt_drivetrain = task(drivetrain_task);
  dt_intake = launch_task(std::bind(&intake::intake_task, &Intake));
  dt_intake_control = task(intake_control_task);
  dt_ladybrown = launch_task(std::bind(&ladybrown::ladybrown_task, &Ladybrown));
  dt_ladybrown_control = task(ladybrown_control_task);
  dt_pnuematic_control = task(pnuematic_control_task);

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
