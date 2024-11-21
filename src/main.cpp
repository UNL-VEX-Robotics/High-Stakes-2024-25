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
motor Left_Motor1 = motor(PORT10, ratio6_1, true);
motor Left_Motor2 = motor(PORT9, ratio6_1, true);
motor Left_Motor3 = motor(PORT8, ratio6_1, true);
motor Left_Motor4 = motor(PORT7, ratio6_1);
motor_group MotorGroupLeft = motor_group(Left_Motor1, Left_Motor2, Left_Motor3, Left_Motor4);

// Motors for Right Side
motor Right_Motor1 = motor(PORT20, ratio6_1);
motor Right_Motor2 = motor(PORT19, ratio6_1);
motor Right_Motor3 = motor(PORT18, ratio6_1);
motor Right_Motor4 = motor(PORT17, ratio6_1, true);
motor_group MotorGroupRight = motor_group(Right_Motor1, Right_Motor2, Right_Motor3, Right_Motor4);

// Intake Motors
motor HookIntake = motor(PORT2, true);
motor FrontIntake = motor(PORT1, true);
motor_group Intake_group = motor_group(HookIntake, FrontIntake);

// Claw Motors
motor ClawMotorLeft = motor(PORT11);
motor ClawMotorRight = motor(PORT12, true);
motor_group ClawMotorGroup = motor_group(ClawMotorLeft, ClawMotorRight);

// Brain and LEDs
vex::brain Brain;
led ClampMotor = led(Brain.ThreeWirePort.A);
led RatchetMotor = led(Brain.ThreeWirePort.C);

inertial Inertial = inertial(PORT3);
Graph graph = Graph(&Brain.Screen);

odometry Odom = odometry(odometry::odometry_pod(odometry::odometry_pod::VERTICAL, &Right_Motor1, 5.656, 0.0212), odometry::odometry_pod(), &Inertial);
chassis Drivetrain = chassis(std::bind(&odometry::getPosition, &Odom), &MotorGroupLeft, &MotorGroupRight, &Inertial, 11.3125, 0.01701696);

// Control Variables
bool toggle = false;
bool wasPressing = false;
bool XwasPressing = false;
bool wasYPressing = false;
bool RightwasPressing = false;
bool clawPresetEnabled = false;
bool redirectMode = false;
bool ejectRing = false;

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
            if (intakeOn || Competition.isAutonomous()) {
                if (Optical.isNearObject()) {
                    Optical.setLight(ledState::on);

                    if ((!isRed && isRedRing(Optical.color())) || (isRed && isBlueRing(Optical.color()))) {
                        ejectRing = true;
                    }

                    if (((isRed && isRedRing(Optical.color())) || (!isRed && isBlueRing(Optical.color()))) && (redirectMode || Competition.isAutonomous())) {
                        redirectRing = true;
                    }
                } else {
                    Optical.setLight(ledState::off);
                }

                if (ejectRing) {
                    if (abs(((int)HookIntake.position(vex::rotationUnits::deg) % ringEjectPosition) - ringEjectPosition) < 35) {
                        task::sleep(50);
                        Intake_group.spin(reverse, 100, percent);
                        task::sleep(200);
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
}

//------------------------------------------------------------------------------
// Autonomous Task
//------------------------------------------------------------------------------

/**
 * @brief Function to handle autonomous control.
 */
/*
void skills(void) {
    redirectRing = true;
    Inertial.setHeading(220, degrees);

    Drivetrain.setDriveConstants(0.95, 0.005, 1, 9, 0.25, 30, -12, 12, 0.5);
    Drivetrain.setTurnConstants(0.128, 0.005, 0.025, 5, 0.05, 50, -12, 12);
    Drivetrain.setSwingConstants(0.25, 0.0, 0.225, 15, 0.5, 50, -12, 12);
    Drivetrain.setArcConstants(0.325, 0.01, 0.7, 15, 0.5, 50, -12, 12);
    vex::thread intake_Functionality = vex::thread(intake_task);
    task::sleep(2000);
    Intake_group.spin(forward, 100, percent);
    ClampMotor.on();


    Drivetrain.swingFor(right, 77.0, 0.7);
    Drivetrain.driveFor(17, 1);
    task::sleep(1000);
    redirectRing = false;
    Drivetrain.driveFor(-24.5);
    
    Drivetrain.turnFor(88, 1);
    Drivetrain.driveFor(-17);
    Intake_group.stop();
    waitUntil(400);
    ClampMotor.off();
    Intake_group.spin(forward, 100, percent);
    Drivetrain.swingFor(right, 190.0, 1.5);
    Drivetrain.driveFor(30 + 3);
    Drivetrain.swingFor(right, 50, 1);
    Drivetrain.driveFor(6, 0.7);
    Drivetrain.driveFor(-3, 0.5);
    ClawMotorGroup.setVelocity(100, percent);
    ClawMotorGroup.spinFor(1400, degrees);
    Drivetrain.driveFor(8, 0.7);
    ClawMotorGroup.spinFor(-(1400 - 600), degrees);
}
*/
void autonomous(void) {
    
    Inertial.setHeading(220, degrees);

    Drivetrain.setDriveConstants(0.95, 0.005, 1, 9, 0.25, 30, -12, 12, 0.5);
    Drivetrain.setTurnConstants(0.128, 0.005, 0.025, 5, 0.05, 50, -12, 12);
    Drivetrain.setSwingConstants(0.25, 0.0, 0.225, 15, 0.5, 50, -12, 12);
    Drivetrain.setArcConstants(0.325, 0.01, 0.7, 15, 0.5, 50, -12, 12);

    ClampMotor.on();
    Drivetrain.driveFor(-14, .5);
    ClampMotor.off();
    Intake_group.spin(forward, 100, percent);
    Drivetrain.swingFor(left, -50, .5);
    ClawMotorGroup.setVelocity(100, percent);
    ClawMotorGroup.spinFor(660, degrees);
    Drivetrain.driveFor(15, 0.7);
    ClawMotorGroup.spinFor(-(660-370), degrees);
    Drivetrain.driveFor(-11);
    
    Drivetrain.swingFor(left, -50, .5);
    Drivetrain.driveFor(20, 1);
    Drivetrain.swingFor(right, 60, 1);
    Drivetrain.driveFor(23, .8);
    waitUntil(750);
    Drivetrain.swingFor(left, 60, .5);
    ClawMotorGroup.spinFor(190, degrees);
    Drivetrain.driveFor(14, 2);
    Drivetrain.driveFor(-7, .7);
    ClawMotorGroup.spinFor(-195, degrees);
    Drivetrain.swingFor(right, -126.5, 1);
    Drivetrain.driveFor(90, 4);
    Drivetrain.turnFor(35, .7);
    Drivetrain.driveFor(20, 2);
    Drivetrain.driveFor(-10, 1);
    Drivetrain.turnFor(190, 2);
    ClampMotor.on();
    Intake_group.stop(hold);
    Drivetrain.driveFor(-10, 1);
    Drivetrain.driveFor(-10, 1);
    ClawMotorGroup.setTimeout(1, seconds);
    ClawMotorGroup.spinFor(800, degrees);
    Drivetrain.driveFor(69, 1);




    


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
        if (Controller.ButtonX.pressing() && !XwasPressing) {
            RatchetMotor.set(!RatchetMotor);
        }

        if (Controller.ButtonR2.pressing()) {
            ClawMotorGroup.spin(reverse, 100, percent);
        }else if (Controller.ButtonR1.pressing()) {
            ClawMotorGroup.spin(forward, 100, percent);
        }
        else{
          ClawMotorGroup.stop();
        }
        

        // Update Toggle States
        RightwasPressing = Controller.ButtonRight.pressing();
        L1WasPressing = Controller.ButtonL1.pressing();
        XwasPressing = Controller.ButtonX.pressing();

        // Set Drivetrain control
        MotorGroupLeft.spin(fwd, Controller.Axis3.position(), percent);
        MotorGroupRight.spin(fwd, Controller.Axis2.position(), percent);
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
