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
#include "drivetrain.h"

using namespace vex;

// A global instance of competition
competition Competition;

//------------------------------------------------------------------------------
// Global Variables and Device Definitions
//------------------------------------------------------------------------------

// Controller
controller Controller = controller();

// Motors for Left Side
motor Left_Motor1 = motor(PORT20, true);
motor Left_Motor2 = motor(PORT14, true);
motor_group MotorGroupLeft = motor_group(Left_Motor1, Left_Motor2);

// Motors for Right Side
motor Right_Motor1 = motor(PORT10);
motor Right_Motor2 = motor(PORT3);
motor_group MotorGroupRight = motor_group(Right_Motor1, Right_Motor2);

// Intake Motors
motor HookIntake = motor(PORT19, true);
motor FrontIntake = motor(PORT3, true);

// Claw Motors
motor ClawMotorLeft = motor(PORT11);
motor ClawMotorRight = motor(PORT12, true);
motor_group ClawMotorGroup = motor_group(ClawMotorLeft, ClawMotorRight);

// Brain and LEDs(motors)
brain Brain;
led ClampMotor = led(Brain.ThreeWirePort.A);
led RatchetMotor = led(Brain.ThreeWirePort.B);

//------------------------------------------------------------------------------
// Pre-Autonomous Functions
//------------------------------------------------------------------------------

/**
 * @brief Function to handle pre-autonomous setup.
 * 
 * This function is called once after the V5 has been powered on.
 * It can be used to initialize sensors, reset encoders, etc.
 */
void pre_auton(void) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

  
}

//------------------------------------------------------------------------------
// Autonomous Task
//------------------------------------------------------------------------------

/**
 * @brief Function to handle autonomous control.
 * 
 * This function should contain the autonomous routine for the robot.
 */
void autonomous(void) {
    /**
    
    
    */

}

/**
 * @brief Function to read the current position from encoders.
 * 
 * @return float Average position from left and right motor encoders.

float readCurrentPosition() {
  // Assuming you have encoders set up, return the average of the left and right motor encoder values.
  float leftPosition = Left_Motor1.rotation(degrees);
  float rightPosition = Right_Motor2.rotation(degrees);
  return (leftPosition + rightPosition) / 2; // Return the average
}
*/
//------------------------------------------------------------------------------
// User Control Task
//------------------------------------------------------------------------------

/**
 * Function to handle user control.
 * 
 * This function should contain the teleoperated control code for the robot.
 */
void usercontrol(void) {

  while (true) {

    // Drive Control (Tank Drive)
    MotorGroupLeft.spin(fwd, Controller.Axis3.position(), percent);
    MotorGroupRight.spin(fwd, Controller.Axis2.position(), percent);

    task::sleep(20);
  }

}


//------------------------------------------------------------------------------
// Main Function
//------------------------------------------------------------------------------

/**
 * @brief Main function to set up competition callbacks and run the robot.
 * 
 * @return int 
 */
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    task::sleep(100);
  }
}
