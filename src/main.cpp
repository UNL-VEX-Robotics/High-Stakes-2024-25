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
#include "pid.h"
#include "stanley.h"

using namespace vex;

// A global instance of competition
competition Competition;

/* ---------- Global Devices ---------- */
//brain
brain Brain;

//controller
controller Controller1 = controller(primary);

//motors
motor Left = motor(PORT1);
motor Right = motor(PORT2, true);

//sensors
inertial Inertial = inertial(PORT10);
encoder Vertical = encoder(Brain.ThreeWirePort.C);
encoder Horizontal = encoder(Brain.ThreeWirePort.A);

//objects
Stanley stanley = Stanley();
odom Odometry(Vertical, Horizontal, Inertial, -0.5, 0.024, 0.875, 0.024, 10);

//tasks
task startOdom;

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/* ---------- Autonomous ---------- */

int start_odometry(){
  Odometry.start();
  return 0;
}

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/* ---------- User Control ---------- */


void usercontrol(void) {
  // User control code here, inside the loop

  Inertial.calibrate();
  vex::wait(3, seconds);

  startOdom = task(start_odometry);
  while (1) {
    Left.spin(forward, Controller1.Axis3.position(percent) + Controller1.Axis1.position(percent), percent);
    Right.spin(forward, Controller1.Axis3.position(percent) - Controller1.Axis1.position(percent), percent);

    vex::wait(20, msec); // Sleep the task for a short amount of time to
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
