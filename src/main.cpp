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

// define your global instances of motors and other devices here
brain Brain;
selector autoSelection = selector(&Brain.Screen, true);


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
  autoSelection.setScreenColor(color(55, 55, 55));
  autoSelection.setPageDefaults(vex::color(white), vex::color(155, 155, 155), vex::color(white), vex::color(55, 55, 55), vex::color(55, 55, 55), vex::color(white), 50, 2);
  autoSelection.setAutonDefaults(vex::color(white), vex::color(white), vex::color(black), vex::color(200, 200, 200), vex::color(200, 200, 200), vex::color(black), 100, 30, 2);
  autoSelection.addPage("Red", vex::color(white), vex::color(155, 155, 155), vex::color(white), vex::color(200, 0, 0), vex::color(200, 0, 0), vex::color(white));
  autoSelection.addPage("Blue", vex::color(white), vex::color(155, 155, 155), vex::color(white), vex::color(0, 0, 200), vex::color(0, 0, 200), vex::color(white));
  autoSelection.addAuton(20, 70, "Positive", "Red");
  autoSelection.addBreak(250, 100, "Calibrate", "none", vex::color(black), vex::color(black), vex::color(white));

  std::vector<const char*> selectedAuto = autoSelection.runSelection();

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Color: ");
  Brain.Screen.print(selectedAuto.at(0));
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("Auto: ");
  Brain.Screen.print(selectedAuto.at(1));
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

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
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
