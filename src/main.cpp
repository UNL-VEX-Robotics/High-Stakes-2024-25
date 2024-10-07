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


// define your global instances of motors and other devices here


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
  /*
  wait(){
    // Motor initialization
    motor Left_Motor1 = motor(PORT1);
    motor Right_Motor2 = motor(PORT2);
    motor_group Motor(Left_Motor1, Right_Motor2);

    // PID parameters
    const float Kp = 1.0;  
    const float Ki = 0.1;  
    const float Kd = 0.01; 
    const float integralTolerance = 0.1;
    const float settleTolerance = 0.1;
    const float settleTime = 1000; 
    const float minOutput = -255;  
    const float maxOutput = 255;    
    const int cycleTime = 100;       

    // Create PID controller object
    PID pid(Kp, Ki, Kd, integralTolerance, settleTolerance, settleTime, minOutput, maxOutput, cycleTime);

    // Initialize variables
    float currentPosition = 0.0;
    float targetDistance = 48.0;
    float error = 0.0;
    bool stopIOvershoot = true; // Prevent integral windup

    // Loop until the desired position is reached
    while (true) {
        // Read the current position from a sensor
        currentPosition = readCurrentPosition(); // Ensure this function is defined to get actual position

        // Calculate the error
        error = targetDistance - currentPosition;

        // Get the PID output
        float output = pid.getOutput(error, stopIOvershoot);

        // Apply the output to the motor group
        Motor.spin(fwd, output, percent);

        // Check if settled
        if (pid.isSettled()) {
            break; // Exit loop if settled
        }

        // Sleep for the cycle time
        task::sleep(cycleTime);
    }
    
    // Stop the motor when done
    Motor.stop();
    }
    */
}

/*
// Define the function to read current position
float readCurrentPosition() {
    // Assuming you have encoders set up, return the average of the left and right motor encoder values.
    float leftPosition = Left_Motor1.rotation(degrees);
    float rightPosition = Right_Motor2.rotation(degrees);
    return (leftPosition + rightPosition) / 2; // Return the average
}
*/


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
    // Initialize controller and motors once
    controller Controller = controller();

    motor Left_Motor1 = motor(PORT13);
    motor Left_Motor2 = motor(PORT14);
    motor Left_Motor3 = motor(PORT15);

    motor_group MotorGroupLeft = motor_group(Left_Motor1, Left_Motor2, Left_Motor3);

    motor Right_Motor1 = motor(PORT16);
    motor Right_Motor2 = motor(PORT17);
    motor Right_Motor3 = motor(PORT18);

    motor_group MotorGroupRight = motor_group(Right_Motor1, Right_Motor2, Right_Motor3);

    motor HookIntake = motor(PORT19);
    motor FrontIntake = motor(PORT20);
    
    motor ClawMotorLeft = motor(PORT11);
    motor ClawMotorRight = motor(PORT5, true);

    motor_group ClawMotorGroup = motor_group(ClawMotorLeft, ClawMotorRight);
    
    brain Brain;
    led clamp = led(Brain.ThreeWirePort.A);
    led ratchet = led(Brain.ThreeWirePort.B);


    while (true) {
        // Control Hook and Front Intake
        if (Controller.ButtonA.pressing()) {
            clamp.on();
        }

         if (Controller.ButtonB.pressing()) {
            ratchet.on();
        }

        if (Controller.ButtonL1.pressing()) {
            HookIntake.spin(fwd, 100, percent);
            FrontIntake.spin(fwd, 100, percent);
        }else if (Controller.ButtonL2.pressing()){
            HookIntake.spin(reverse, 100, percent);
            FrontIntake.spin(reverse, 100, percent);
        } else {
            HookIntake.stop(); // Stop when no button is pressed
            FrontIntake.stop();
        }

        if(Controller.ButtonR1.pressing()){
          ClawMotorGroup.spin(fwd, 100, percent);
        } else if (Controller.ButtonR2.pressing()){
          ClawMotorGroup.spin(reverse, 100, percent);
        } else {
          ClawMotorGroup.stop();
        }

        // Drive Control
        /*
        MotorGroupLeft.spin(fwd, Controller.Axis1.position() - Controller.Axis3.position(), percent);
        MotorGroupRight.spin(fwd, Controller.Axis3.position() + Controller.Axis1.position(), percent);
        */
        
        //tank drive
        MotorGroupLeft.spin(fwd, Controller.Axis3.position(), percent);
        MotorGroupRight.spin(fwd, Controller.Axis2.position(), percent);

        // Wait for a short amount of time to prevent wasted resources
        wait(20, msec);
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
