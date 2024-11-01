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
motor_group IntakeGroup = motor_group(LeftIntake);

motor LeftLift = motor(PORT8, ratio36_1, true);

motor_group Right = motor_group(RightFront, RightMiddle, RightTop, RightBack);
motor_group Left = motor_group(LeftFront, LeftMiddle, LeftTop, LeftBack);

inertial Inertial = inertial(PORT4);
optical Optical = optical(PORT20);

/* ---------- Tasks ---------- */
vex::task dt_drivetrain;
vex::task dt_intake;

vex::task at_intake;

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

int tempReplacementForLadyBrownPositionUntilThatIsActuallyAddedIntoCodeAndIGuessUntilTheLadyBrownMechIsActuallyBuilt(){ return 0; }

/* ---------- Objects ---------- */
intake Intake = intake(&IntakeGroup, &Optical, 1920, tempReplacementForLadyBrownPositionUntilThatIsActuallyAddedIntoCodeAndIGuessUntilTheLadyBrownMechIsActuallyBuilt, -50);
odometry Odom = odometry(odometry::odometry_pod(odometry::odometry_pod::VERTICAL, &LeftFront, 5.65625, 0.0212712), odometry::odometry_pod(), &Inertial);

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
  //after selection
  IntakeGroup.setPosition(0, degrees);
  Intake.setBrakeType(brake);
  Intake.setColor(red);

  Inertial.calibrate();
  do {
    task::sleep(50);
  } while (Inertial.isCalibrating());
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
 * @brief Stops the intake
 * 
 * @param stoppingType  the desired brakeType
 */
void stopIntake(brakeType stoppingType = brakeType::brake)
{
  at_intake.stop();
  LeftIntake.stop(stoppingType);
}

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
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
    Right.spin(forward, Controller1.Axis3.position(percent) - Controller1.Axis1.position(percent), percent);

    task::sleep(10);
  }
}

int intake_control_task()
{
  bool L1_wasPressing = false;
  bool L2_wasPressing = false;

  bool intakeOn = false;
  bool spinForward = true;

  Intake.setColorSort(true);
  while(true)
  {
    if(Controller1.ButtonL1.pressing() && !L1_wasPressing) intakeOn = !intakeOn;
    if(Controller1.ButtonL2.pressing() && !L2_wasPressing) spinForward = !spinForward;

    if(intakeOn && spinForward) Intake.setSpeed(100);
    else if(intakeOn) Intake.setSpeed(-100);
    else Intake.setSpeed(0);

    L1_wasPressing = Controller1.ButtonL1.pressing();
    L2_wasPressing = Controller1.ButtonL2.pressing();
    task::sleep(10);
  }
}

void usercontrol(void) 
{
  wait(3, seconds);
  dt_drivetrain = task(drivetrain_task);
  dt_intake = launch_task(std::bind(&intake::intake_task, &Intake));

  LeftLift.stop(hold);

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
