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
#include "grapher.h"

using namespace vex;

// Global instance of competition
competition Competition;

//------------------------------------------------------------------------------
// Global Variables and Device Definitions
//------------------------------------------------------------------------------

controller Controller = controller();

// Motors for Left Side
motor Left_Motor1 = motor(PORT3, ratio6_1, true);
motor Left_Motor2 = motor(PORT4, ratio6_1, true);
motor Left_Motor3 = motor(PORT6, ratio6_1, true);
motor Left_Motor4 = motor(PORT5, ratio6_1);
motor_group MotorGroupLeft = motor_group(Left_Motor1, Left_Motor2, Left_Motor3, Left_Motor4);

// Motors for Right Side
motor Right_Motor1 = motor(PORT10, ratio6_1);
motor Right_Motor2 = motor(PORT7, ratio6_1);
motor Right_Motor3 = motor(PORT8, ratio6_1);
motor Right_Motor4 = motor(PORT9, ratio6_1, true);
motor_group MotorGroupRight = motor_group(Right_Motor1, Right_Motor2, Right_Motor3, Right_Motor4);

// Intake Motors
motor HookIntake = motor(PORT2, true);
motor FrontIntake = motor(PORT1);
motor_group Intake_group = motor_group(HookIntake, FrontIntake);

// Lady Brown motors just named as Claw Motors
motor ClawMotorLeft = motor(PORT11);
motor ClawMotorRight = motor(PORT12, true);
motor_group ClawMotorGroup = motor_group(ClawMotorLeft, ClawMotorRight);

//arm motors
motor arm_Left = motor(PORT14);
motor arm_Right = motor(PORT13, true);

motor_group ArmMotorGroup = motor_group(arm_Left, arm_Right);


// Brain and LEDs
vex::brain Brain;
led ClampMotor = led(Brain.ThreeWirePort.G);


inertial Inertial = inertial(PORT20);
Graph graph = Graph(&Brain.Screen);

odometry Odom = odometry(odometry::odometry_pod(odometry::odometry_pod::VERTICAL, &Right_Motor1, 5.656, 0.0212), odometry::odometry_pod(), &Inertial);
chassis Drivetrain = chassis(std::bind(&odometry::getPosition, &Odom), &MotorGroupLeft, &MotorGroupRight, &Inertial, 11.3125, 0.0212712);

// Control Variables
bool toggle = false;
bool wasPressing = false;
bool isReverse = false;
bool XwasPressing = false;
bool wasYPressing = false;
bool RightwasPressing = false;
bool clawPresetEnabled = false;
bool redirectMode = false; //will delete
bool ejectRing = false; //will delete
bool YwasPressing = false;

// Optical sensor
optical Optical = optical(PORT4);
double hue = Optical.hue();
bool isRed = true;
bool intakeOn = false;
bool L1WasPressing = false;
bool redirectRing = false;



// Functions to determine ring colors
bool isRedRing(vex::color c) {
    return c == red;
}

bool isBlueRing(vex::color c) {
    return c == blue;
}


//------------------------------------------------------------------------------
// Threading and Tasks
//------------------------------------------------------------------------------

vex::thread redirectThread;


int ringEjectPosition = 1437 / 3;
int ringRedirectPosition = 1437 / 3;

// Intake task
int intake_task() {

    Optical.setLightPower(50, percent);
    Optical.integrationTime(5);

    while (true) {
        if (Controller.ButtonUp.pressing() && !wasPressing) intakeOn = !intakeOn;
        wasPressing = Controller.ButtonUp.pressing();

        if (Controller.ButtonY.pressing() && !wasYPressing) {
            redirectMode = !redirectMode; // Toggle redirect mode
        }
        wasYPressing = Controller.ButtonY.pressing();

        if (Controller.ButtonDown.pressing()) {
            Intake_group.spin(reverse, 100, percent);
        } else {
            if (intakeOn) {
                if (Optical.isNearObject()) {
                    Optical.setLight(ledState::on);

                    if ((!isRed && isRedRing(Optical.color())) || (isRed && isBlueRing(Optical.color()))) {
                        ejectRing = true;
                    }

                    if (((isRed && isRedRing(Optical.color())) || (!isRed && isBlueRing(Optical.color()))) && (redirectMode)) {
                        redirectRing = true;
                    }
                } else {
                    Optical.setLight(ledState::off);
                }

                if (ejectRing) {
                    if (abs(((int)HookIntake.position(vex::rotationUnits::deg) % ringEjectPosition) - ringEjectPosition) < 35) {
                        if(Competition.isAutonomous()){
                            task::sleep(50);
                            task::sleep(20);
                            Intake_group.spin(reverse, 100, percent);
                            task::sleep(120);
                            Intake_group.spin(forward, 100, percent);
                            ejectRing = false;
                        }
                        task::sleep(0);
                        Intake_group.spin(reverse, 100, percent);
                        task::sleep(120);
                        Intake_group.spin(forward, 100, percent);
                        ejectRing = false;
                    }
                } else {
                    Intake_group.spin(forward, 100, percent);
                }

                if (redirectRing) {
                    if (abs((((int)HookIntake.position(deg)) % ringRedirectPosition) - (ringRedirectPosition - 40)) < 35) {
                        task::sleep(27);
                        Intake_group.spin(reverse, 100, percent);
                        task::sleep(1000);
                        Intake_group.spin(forward, 100, percent);
                        redirectRing = false;
                        redirectMode = false;
                    }
                } else {
                    Intake_group.spin(forward, 100, percent);
                }
            } else {
                Intake_group.stop(brake);
            }
        }

        task::sleep(5);
    }
}


//------------------------------------------------------------------------------
// Pre-Autonomous Functions
//------------------------------------------------------------------------------

/**
 * @brief Function to handle pre-autonomous setup.
 */
void pre_auton(void) {
    
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.setPenColor(white);
    Brain.Screen.setFillColor(black);
    Brain.Screen.print("Calibrating...");

    Inertial.startCalibration();
    do {
        task::sleep(50);
    } while (Inertial.isCalibrating());

    Brain.Screen.setCursor(2, 1);
    Brain.Screen.setPenColor(green);
    Brain.Screen.print("Calibrated.");
    task::sleep(10);


    Brain.Screen.setPenColor(red);
    Brain.Screen.newLine();
    
    Inertial.setHeading(270, degrees);
    Drivetrain.setDriveConstants(0.95, 0.005, 1, 9, 0.75, 30, -12, 12, 0.5);
    Drivetrain.setTurnConstants(0.20, 0.01, 0.48, 15, 0.5, 50, -12, 12);
    Drivetrain.setSwingConstants(0.2, 0.005, 0.3, 22, 0.5, 50, -12, 12);
    Drivetrain.setArcConstants(0.25, 0.01, 0.7, 15, 0.5, 50, -12, 12);
}

//------------------------------------------------------------------------------
// Autonomous Task
//------------------------------------------------------------------------------

/**
 * @brief Function to handle autonomous control.
 */

void skills(void) {

Inertial.setHeading(270, degrees);
    Drivetrain.setDriveConstants(1.15, 0.005, 0.6 , 9, 0.75, 30, -12, 12, 0.5);
    Drivetrain.setTurnConstants(0.18, 0.01, 0.48, 15, 0.5, 50, -12, 12);
    Drivetrain.setSwingConstants(0.2, 0.005, 0.3, 22, 0.5, 50, -12, 12);
    Drivetrain.setArcConstants(0.25, 0.01, 0.7, 15, 0.5, 50, -12, 12);
    //Drivetrain.driveFor(24);
    //Drivetrain.turnFor(90);
    //Drivetrain.swingFor(right,90);
    ClawMotorGroup.stop(hold);
    ArmMotorGroup.stop(hold);
    Intake_group.spin(forward, 50, percent);
    Drivetrain.driveFor(32,2);
    wait(0.15,seconds);
    Intake_group.stop(brake);
    Drivetrain.turnFor(-77);
    wait(.25,seconds);
    Drivetrain.driveFor(-17);
    Drivetrain.driveFor(-7);
    wait(.25, seconds);
    ClampMotor.set(!ClampMotor);
    wait(0.15, seconds);
    Intake_group.spin(forward, 100, percent);
    wait(0.75, seconds);
    Drivetrain.driveFor(22);
    Drivetrain.turnFor(47);
    Intake_group.stop(brake);
    Drivetrain.driveFor(13);
    Drivetrain.driveFor(6);
    Intake_group.spin(forward, 100, percent);
    Drivetrain.driveFor(5);
    wait(0.25, seconds);
    Intake_group.stop(brake);
    Drivetrain.driveFor(-30);
    Intake_group.spinFor(reverse, 0.5, seconds);
    wait(0.25, seconds);
    //ArmMotorGroup.spinFor(561, degrees);
    //ClawMotorGroup.spinFor(-160, degrees);
    Intake_group.spin(forward,100,percent);
    wait(0.5, seconds);
    Drivetrain.turnFor(135);
    ArmMotorGroup.spinFor(200, degrees);
    Drivetrain.driveFor(180,2);
    Drivetrain.driveFor(-3);
    wait(0.5, seconds);
    Intake_group.stop(brake);
    Drivetrain.turnFor(-87);
    Intake_group.spin(forward, 100, percent);
    Drivetrain.driveFor(25);
    Drivetrain.swingFor(left, 185);
    Intake_group.spin(forward, 100, percent);
    Drivetrain.driveFor(180,1.75);
    Drivetrain.driveFor(-15);
    Drivetrain.turnFor(-45);
    Intake_group.stop(brake);
    Drivetrain.driveFor(20, 1);
    wait(0.5, seconds);
    ArmMotorGroup.spinFor(361, degrees);
    ClawMotorGroup.spinFor(-160, degrees);
    Intake_group.spin(forward, 100, percent);
    Drivetrain.driveFor(-15, 1);
    Drivetrain.turnFor(-180, 1);
    wait(.5, seconds);
    ClampMotor.on();
    Drivetrain.driveFor(-20, 1);
    Drivetrain.driveFor(32);
    Drivetrain.turnFor(45);
    Intake_group.stop(brake);
    Drivetrain.driveFor(-45, 1.5);
    Drivetrain.driveFor(48);
    Drivetrain.turnFor(87);
    Drivetrain.driveFor(45, 0.1);
    Drivetrain.driveFor(-24);
    Drivetrain.turnFor(-87);


}

void match(void) {
    Inertial.setHeading(220, degrees);    
    vex::thread intake_Functionality = vex::thread(intake_task);


}
void autonomous(void) {
    
    //match();
    skills();

    



    


}

//------------------------------------------------------------------------------
// User Control Task
//------------------------------------------------------------------------------

/**
 * @brief Function to handle user control.
 */
void usercontrol(void) {
    vex::thread intake_Functionality = vex::thread(intake_task);

    while (true) {
        // Toggle Clamp LED on ButtonL1 Press
        if (Controller.ButtonL1.pressing() && !L1WasPressing) {
            ClampMotor.set(!ClampMotor);
        }

        // Toggle RatchetMotor LED on ButtonX Press

        if (Controller.ButtonX.pressing()) {
            ClawMotorGroup.spin(reverse, 100, percent);
        }else if (Controller.ButtonB.pressing()) {
            ClawMotorGroup.spin(forward, 100, percent);
        }
        else{
          ClawMotorGroup.stop(brake);
        }

        if (Controller.ButtonR2.pressing()) {
            ArmMotorGroup.spin(fwd, 100, percent);
        }
        else if (Controller.ButtonR1.pressing()) {
            ArmMotorGroup.spin(reverse, 100, percent);
        }
        else{
            ArmMotorGroup.stop(brake);
        }
        

        // Update Toggle States
        RightwasPressing = Controller.ButtonRight.pressing();
        L1WasPressing = Controller.ButtonL1.pressing();
        YwasPressing = Controller.ButtonY.pressing();

        // Set Drivetrain control
        MotorGroupLeft.spin(fwd, Controller.Axis3.position() + Controller.Axis1.position(), percent);
        MotorGroupRight.spin(fwd, Controller.Axis3.position() - Controller.Axis1.position() , percent);
        task::sleep(20);
    }
}

//------------------------------------------------------------------------------
// Main Entry Point
//------------------------------------------------------------------------------

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
