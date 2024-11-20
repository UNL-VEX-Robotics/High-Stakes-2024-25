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
led RatchetMotor = led(Brain.ThreeWirePort.B);

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
bool isRed = false;



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


int ringEjectPosition = 1430 / 3;
int ringRedirectPosition = 1430 / 3;


// Intake task
int intake_task() {
    bool L1WasPressing = false;
    bool intakeOn = false;
    bool ejectRing = false;
    bool redirectRing = false;

    Optical.setLightPower(50, percent);
    Optical.integrationTime(5);

    while (true) {
        if (Controller.ButtonL1.pressing() && !L1WasPressing) intakeOn = !intakeOn;
        L1WasPressing = Controller.ButtonL1.pressing();

        if (Controller.ButtonY.pressing() && !wasYPressing) {
            redirectMode = !redirectMode; // Toggle redirect mode
        }
        wasYPressing = Controller.ButtonY.pressing();

        if (Controller.ButtonL2.pressing()) {
            Intake_group.spin(reverse, 100, percent);
        } else {
            if (intakeOn) {
                if (Optical.isNearObject()) {
                    Optical.setLight(ledState::on);

                    if ((!isRed && isRedRing(Optical.color())) || (isRed && isBlueRing(Optical.color()))) {
                        ejectRing = true;
                    }

                    if (((isRed && isRedRing(Optical.color())) || (!isRed && isBlueRing(Optical.color()))) && redirectMode) {
                        redirectRing = true;
                    }
                } else {
                    Optical.setLight(ledState::off);
                }

                if (ejectRing) {
                    if (abs(((int)HookIntake.position(vex::rotationUnits::deg) % ringEjectPosition) - ringEjectPosition) < 35) {
                        task::sleep(200);
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
void autonomous(void) {
    Inertial.setHeading(220, degrees);

    Drivetrain.setDriveConstants(0.95, 0.005, 1, 9, 0.25, 30, -12, 12, 0.5);
    Drivetrain.setTurnConstants(0.128, 0.005, 0.025, 5, 0.05, 50, -12, 12);
    Drivetrain.setSwingConstants(0.25, 0.0, 0.225, 15, 0.5, 50, -12, 12);
    Drivetrain.setArcConstants(0.325, 0.01, 0.7, 15, 0.5, 50, -12, 12);
    Intake_group.spin(forward, 100, percent);
    ClampMotor.on();

    task::sleep(250);

    Drivetrain.swingFor(right, 77.0, 0.7);
    Drivetrain.driveFor(17, 1);
    Drivetrain.driveFor(-24.5);
    Drivetrain.turnFor(88, 1);
    Drivetrain.driveFor(-14.5);
    Intake_group.stop();
    waitUntil(400);
    ClampMotor.off();
    Intake_group.spin(forward, 100, percent);
    Drivetrain.swingFor(right, 190.0, 1.5);
    Drivetrain.driveFor(26.5);
    Drivetrain.swingFor(right, 45, 1);
    Drivetrain.driveFor(5, 0.7);
    Drivetrain.driveFor(-3, 0.5);
    ClawMotorGroup.setVelocity(100, percent);
    ClawMotorGroup.spinFor(1400, degrees);
    Drivetrain.driveFor(6, 0.7);
    ClawMotorGroup.spinFor(-(1400 - 600), degrees);
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
