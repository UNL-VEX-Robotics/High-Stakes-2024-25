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

motor LeftLift = motor(PORT8, ratio36_1, true);

motor_group Right = motor_group(RightFront, RightMiddle, RightTop, RightBack);
motor_group Left = motor_group(LeftFront, LeftMiddle, LeftTop, LeftBack);

inertial Inertial = inertial(PORT4);
optical Optical = optical(PORT20);

/* ---------- Tasks ---------- */
vex::task dt_drivetrain;
vex::task dt_intake;

/* ---------- Global Variables ---------- */
bool isRed = false;

const int ringEjectPosition = 1920; //should be 650 once there is 3 hooks, start code with a hook vertical

/**
 * @breif determines if the ring is red based on hue
 * 
 * @param hue the hue seen by the optical sensor
 * 
 * @return  true if red, false otherwise
 */
bool isRedRing(vex::color c)
{
  if(c == red) return true;
  return false;
}

/**
 * @breif determines if the ring is blue based on hue
 * 
 * @param hue the hue seen by the optical sensor
 * 
 * @return  true if blue, false otherwise
 */
bool isBlueRing(vex::color c)
{
  if(c == blue) return true;
  return false;
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

int drivetrain_task()
{
  while(true)
  {
    Left.spin(forward, Controller1.Axis3.position(percent) + Controller1.Axis1.position(percent), percent);
    Right.spin(forward, Controller1.Axis3.position(percent) - Controller1.Axis1.position(percent), percent);

    task::sleep(10);
  }
}

int intake_task()
{
  bool R1WasPressing = false;

  bool intakeOn = false;
  bool ejectRing = false;

  Optical.setLightPower(50, percent);
  Optical.integrationTime(5);
  while(true)
  {
    if(Controller1.ButtonR1.pressing() && !R1WasPressing) intakeOn = !intakeOn;
    R1WasPressing = Controller1.ButtonR1.pressing();

    if(Controller1.ButtonR2.pressing()) LeftIntake.spin(reverse, 100, percent);
    else
    {
      if (intakeOn) 
      {
        if(Optical.isNearObject())
        {
          Optical.setLight(ledState::on);

          if((isRedRing(Optical.color()) && !isRed) || (isBlueRing(Optical.color()) && isRed)) ejectRing = true;
        }
        else Optical.setLight(ledState::off);

        if(ejectRing)
        {
          if (abs(((int)LeftIntake.position(deg) % ringEjectPosition) - ringEjectPosition) < 35)
          {
            LeftIntake.spin(reverse, 100, percent);
            task::sleep(250);
            LeftIntake.spin(forward, 100, percent);
            ejectRing = false;
          }
        }
        else LeftIntake.spin(forward, 100, percent);
      }
      else LeftIntake.stop(brake);
    }

    task::sleep(5);
  }
}

void usercontrol(void) 
{
  dt_drivetrain = task(drivetrain_task);
  dt_intake = task(intake_task);

  LeftLift.stop(hold);

  while (1) 
  {
    waitUntil(Brain.Screen.pressing());
    waitUntil(!Brain.Screen.pressing());

    isRed = !isRed;

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
