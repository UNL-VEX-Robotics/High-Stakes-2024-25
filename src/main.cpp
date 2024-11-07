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
motor Left_Motor1 = motor(PORT4, ratio6_1, true);
motor Left_Motor2 = motor(PORT14, ratio6_1, true);
motor Left_Motor3 = motor(PORT5, ratio6_1, true);
motor Left_Motor4 = motor(PORT20, ratio6_1);
motor_group MotorGroupLeft = motor_group(Left_Motor1, Left_Motor2, Left_Motor3, Left_Motor4);

// Motors for Right Side
motor Right_Motor1 = motor(PORT7, ratio6_1);
motor Right_Motor2 = motor(PORT10, ratio6_1);
motor Right_Motor3 = motor(PORT18, ratio6_1);
motor Right_Motor4 = motor(PORT8, ratio6_1, true);
motor_group MotorGroupRight = motor_group(Right_Motor1, Right_Motor2, Right_Motor3, Right_Motor4);

// Intake Motors
motor HookIntake = motor(PORT19, true);
motor FrontIntake = motor(PORT3, true);

motor_group Intake_group = motor_group(HookIntake, FrontIntake);


// Claw Motors
motor ClawMotorLeft = motor(PORT11);
motor ClawMotorRight = motor(PORT12, true);
motor_group ClawMotorGroup = motor_group(ClawMotorLeft, ClawMotorRight);

// Brain and LEDs(motors)
brain Brain;
led ClampMotor = led(Brain.ThreeWirePort.A);
led RatchetMotor = led(Brain.ThreeWirePort.B);

inertial Inertial = inertial(PORT1);

odometry Odom = odometry(odometry::odometry_pod(odometry::odometry_pod::VERTICAL, &MotorGroupLeft, 5.656, 0.0212), odometry:odometry_pod(), &Inertial);
chassis Drivetrain = chassis(std::bind(&odometry::getPosition, &Odom), &MotorGroupLeft, &MotorGroupRight, &Intertial, 11.3125, 0.0212712 ); 
//rotation ClawRotationSensor = rotation(PORTX); 


// Control Variables
bool toggle = false;
bool wasPressing = false;
bool XwasPressing = false;
bool wasYPressing = false;
bool RightwasPressing = false;
bool clawPresetEnabled = false;


bool redirectMode = false;
bool ejectRing = false;

bool isRedRing(vex::color c)
{
  if(c == red){
    return true;
  } 
  return false;
}

bool isBlueRing(vex::color c)
{
  if(c == blue){
    return true;
  } 
  return false;
}

const int ringEjectPosition = (1440/3); 

const int ringRedirectPostion = (1440/3);


//Limit swtich 
//limit limitSwitch = limit(Brain.ThreeWirePort.H);

//Optiic sensor
optical Optical = optical(PORT2);
double hue = Optical.hue();
//optical mode
bool isRed = false;

//threading
vex::thread redirectThread;

int intake_task()
{
  bool L1WasPressing = false;

  bool intakeOn = false;
  bool ejectRing = false;
  bool redirectRing = false; 

  Optical.setLightPower(50, percent);
  Optical.integrationTime(5);
  while(true)
  {
    if(Controller.ButtonL1.pressing() && !L1WasPressing) intakeOn = !intakeOn;
    L1WasPressing = Controller.ButtonL1.pressing();

    if (Controller.ButtonY.pressing() && !wasYPressing) {
      redirectMode = !redirectMode; // Toggle redirect mode on button press 
      }
      wasYPressing = Controller.ButtonY.pressing(); // Update previous state of Y button
     

    if(Controller.ButtonL2.pressing()) Intake_group.spin(reverse, 100, percent);
    else
    {
      if (intakeOn) 
      {
        if(Optical.isNearObject())
        {
          Optical.setLight(ledState::on);

          if((!isRed && isRedRing(Optical.color())) || (isRed && isBlueRing(Optical.color()))){
            ejectRing = true;
          } 

          if(((isRed && isRedRing(Optical.color())) || (!isRed && isBlueRing(Optical.color()))) && redirectMode){
            redirectRing= true;
          }
        }
        else Optical.setLight(ledState::off);

        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print(abs((((int)HookIntake.position(deg) - 5) % ringEjectPosition) - ringEjectPosition));

        if(ejectRing)
        {
          if (abs((((int)HookIntake.position(deg)) % ringEjectPosition) - ringEjectPosition) < 35)
          {
            Intake_group.stop(hold);
            task::sleep(300);
            Intake_group.spin(forward, 100, percent);
            ejectRing = false;
          }
        }
        else Intake_group.spin(forward, 100, percent);

        if(redirectRing){
          if (abs((((int)HookIntake.position(deg))% ringRedirectPostion) - ringRedirectPostion) < 35)
          {
            Intake_group.spin(reverse, 100, percent);
            task::sleep(600);
            Intake_group.spin(forward, 100, percent);
            redirectRing = false;
          }
        }
        else Intake_group.spin(forward, 100, percent);


      }
      else Intake_group.stop(brake);
    }

    task::sleep(5);
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

  Optical.setLightPower(100, percent);
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
  Drivetrain.setDriveConstants(.75, .005, 1, 9, .75, 30, -12, 12, .5);
  Drivetrain.setTurnConstants(.15, .01, .6, 15, .05, 50, -12, 12);
  Drivetrain.setSwingConstants(.2, .005, .3, 15, .5, 50, -12, 12);
  Drivetrain.setArcConstants(.25, .001, 7, 15, .5, 50, -12, 12);

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
  vex::thread intake_Functionality  = vex::thread(intake_task);
  
  while (true) {
    
    // Toggle Clamp LED on ButtonL1 Press
    if (Controller.ButtonUp.pressing() && !wasPressing) {
      ClampMotor.set(!ClampMotor);
    }

    // Toggle RatchetMotor LED on ButtonX Press
    if (Controller.ButtonX.pressing() && !XwasPressing) {
      RatchetMotor.set(!RatchetMotor);
    }


    if (Controller.ButtonRight.pressing() && !RightwasPressing) {
      ClawMotorGroup.spin(fwd, 100, dps);
    }

    


    // Update Toggle States
    RightwasPressing = Controller.ButtonRight.pressing();
    wasPressing = Controller.ButtonUp.pressing();
    XwasPressing = Controller.ButtonX.pressing();

    
    if (Controller.ButtonRight.pressing() && !RightwasPressing) {
      clawPresetEnabled = !clawPresetEnabled; // Toggle the preset
      if (clawPresetEnabled) {
        ClawMotorGroup.spin(fwd, 90, dps); // Move to preset angle
      } else {
        ClawMotorGroup.spin(reverse, 90, dps); // Return to starting position
      }
    }

    // Control Claw Motors
    if (Controller.ButtonR2.pressing()) {
      ClawMotorGroup.spin(fwd, 100, percent);
    } else if (Controller.ButtonR1.pressing()) {
      ClawMotorGroup.spin(reverse, 100, percent);
    } else {
      ClawMotorGroup.stop();
    }      
    
   // MotorGroupLeft.spin(fwd, Controller.Axis3.position(), percent);
   // MotorGroupRight.spin(fwd, Controller.Axis2.position(), percent);



  if (Controller.Axis3.position() == 0 && Controller.Axis2.position() == 0) {
      // If no input, motors hold position (brake hold)
      MotorGroupLeft.stop(brake);
      MotorGroupRight.stop(brake);
  } else {
      // Otherwise, spin motors based on controller input
      MotorGroupLeft.spin(fwd,  Controller.Axis3.position(), percent);
      MotorGroupRight.spin(fwd,  Controller.Axis2.position(), percent);
  }

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