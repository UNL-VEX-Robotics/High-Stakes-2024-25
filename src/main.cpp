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
motor Left_Motor1 = motor(PORT4, true);
motor Left_Motor2 = motor(PORT14, true);
motor Left_Motor3 = motor(PORT1, true);
motor_group MotorGroupLeft = motor_group(Left_Motor1, Left_Motor2, Left_Motor3);

// Motors for Right Side
motor Right_Motor1 = motor(PORT7);
motor Right_Motor2 = motor(PORT10);
motor Right_Motor3 = motor(PORT18);
motor_group MotorGroupRight = motor_group(Right_Motor1, Right_Motor2, Right_Motor3);

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

// Control Variables
bool toggle = false;
bool wasPressing = false;
bool L1wasPressing = false;
bool XwasPressing = false;
bool wasYPressing = false; 
bool redirectMode = false;


//Limit swtich 
//limit limitSwitch = limit(Brain.ThreeWirePort.H);

//Optiic sensor
optical Optical17 = optical(PORT17);
double hue = Optical17.hue();
//optical mode
bool isRed = true;

//threading
vex::thread redirectThread;


void intake_Functionality() {
  while(true){

    // Toggle Intake Motors on Button Up Press
    if (Controller.ButtonL1.pressing() && !L1wasPressing) {
      toggle = !toggle;
    }

    // Hold-to-Reverse functionality for Intake Motors (Button Down)
    if (Controller.ButtonL2.pressing()) {
      HookIntake.spin(reverse, 100, percent);
      FrontIntake.spin(reverse, 100, percent);
    } else if (toggle) {
      // Run Intake Motors Forward if toggled
      HookIntake.spin(fwd, 100, percent);
      FrontIntake.spin(fwd, 100, percent);
    } else {
      // Stop Intake Motors if not toggled and Button Down not pressed
      HookIntake.stop();
      FrontIntake.stop();
    }

    // Check for Y button press to toggle redirect mode

    if (Controller.ButtonY.pressing() && !wasYPressing) {
      redirectMode = !redirectMode; // Toggle redirect mode on button press
    }
    wasYPressing = Controller.ButtonY.pressing(); // Update previous state of Y button
    L1wasPressing = Controller.ButtonL1.pressing();


    // If in redirect mode and the obtical senses color 
    if (Optical17.isNearObject()) {
      Optical17.setLightPower(100);
      Optical17.setLight(ledState::on);
      
      if (Optical17.hue() > 180 && Optical17.hue() < 240) {
        float targetPosition = HookIntake.position(degrees) + 58; // Fine-tune this value
        float backTargetPosition = targetPosition - 3;

        // Wait until the intake reaches the desired position, then reverse to eject
        if (Optical17.hue() > 180 && Optical17.hue() < 240 && HookIntake.position(degrees) < targetPosition){
          waitUntil(HookIntake.position(degrees) > targetPosition);
        }
        waitUntil(HookIntake.position(degrees) > targetPosition);
        HookIntake.spin(reverse, 100, percent);
        FrontIntake.spin(reverse, 100, percent);

        // Wait until the intake returns to the starting position
        waitUntil(HookIntake.position(degrees) < backTargetPosition);
      }




      
      if(((isRed && Optical17.hue() > 0 && Optical17.hue() < 30) \
      || (!isRed && Optical17.hue() > 180 && Optical17.hue() < 240)) && redirectMode){
        float targetPosition = HookIntake.position(degrees) + 20.215; //tuneable
        float backTargetPosition = HookIntake.position(degrees) - 350;
        waitUntil(HookIntake.position(degrees) > targetPosition);
        HookIntake.spin(reverse, 100, percent); // Spin the hook intake
        FrontIntake.spin(reverse, 100, percent); // Spin the front intake
        waitUntil(HookIntake.position(degrees) < backTargetPosition);
        redirectMode = false; // Optionally reset redirect mode after action
        }
    }
    else{
      Optical17.setLight(ledState::off);
    }
    
    vex::this_thread::sleep_for(10);  // Small delay 
  }
}


//redirect function (for threading)
/*
void redirectMotor() {

  // Perform redirection logic here
    float targetPosition = HookIntake.position(degrees) + 58; // Adjust as needed
    float backTargetPosition = HookIntake.position(degrees) - 5;

    // Redirect forward until the target position
    while (HookIntake.position(degrees) < targetPosition) {
      HookIntake.spin(fwd, 100, percent);
      FrontIntake.spin(fwd, 100, percent);
      this_thread::sleep_for(10);  // Small delay to prevent CPU overload
    }

    // Reverse the motors for the specified distance
    while (HookIntake.position(degrees) > backTargetPosition) {
      HookIntake.spin(reverse, 100, percent);
      FrontIntake.spin(reverse, 100, percent);
      this_thread::sleep_for(10);
    

    // Stop the motors when the redirection is done
    HookIntake.stop();
    FrontIntake.stop();
  }
}
*/ 


//eject function (used in threading)

/*
void Eject(){
        float targetPosition = HookIntake.position(degrees) + 27; //tuneable
        float backTargetPosition = HookIntake.position(degrees) - 300;
        waitUntil(HookIntake.position(degrees) > targetPosition);
        HookIntake.spin(reverse, 100, percent); // Spin the hook intake
        FrontIntake.spin(reverse, 100, percent); // Spin the front intake
        //task::sleep(2000); // Wait for 2 seconds
        waitUntil(HookIntake.position(degrees) < backTargetPosition);
        redirectMode = false; // Optionally reset redirect mode after action
}
*/
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

  Optical17.setLightPower(100, percent);
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
  //thread
  vex::thread t_intake = vex::thread(intake_Functionality);
  
  while (true) {
    
    // Toggle Clamp LED on ButtonL1 Press
    if (Controller.ButtonUp.pressing() && !wasPressing) {
      ClampMotor.set(!ClampMotor);
    }

    // Toggle RatchetMotor LED on ButtonX Press
    if (Controller.ButtonX.pressing() && !XwasPressing) {
      RatchetMotor.set(!RatchetMotor);
    }

    


    // Update Toggle States
    wasPressing = Controller.ButtonUp.pressing();
    XwasPressing = Controller.ButtonX.pressing();

    // Control Claw Motors
    if (Controller.ButtonR2.pressing()) {
      ClawMotorGroup.spin(fwd, 100, percent);
    } else if (Controller.ButtonR1.pressing()) {
      ClawMotorGroup.spin(reverse, 100, percent);
    } else {
      ClawMotorGroup.stop();
    }

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