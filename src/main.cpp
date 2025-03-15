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
#include "auton-selector.h"

using namespace vex;

// Global instance of competition
competition Competition;

//------------------------------------------------------------------------------
// Global Variables and Device Definitions
//------------------------------------------------------------------------------

controller Controller = controller();

// Motors for Left Side
motor Left_Motor1 = motor(PORT9, ratio6_1, true);
motor Left_Motor2 = motor(PORT6, ratio6_1, true);
motor Left_Motor3 = motor(PORT10, ratio6_1, true);
motor Left_Motor4 = motor(PORT8, ratio6_1);
motor_group MotorGroupLeft = motor_group(Left_Motor1, Left_Motor2, Left_Motor3, Left_Motor4);

// Motors for Right Side
motor Right_Motor1 = motor(PORT3, ratio6_1);
motor Right_Motor2 = motor(PORT5, ratio6_1);
motor Right_Motor3 = motor(PORT4, ratio6_1);
motor Right_Motor4 = motor(PORT2, ratio6_1, true);
motor_group MotorGroupRight = motor_group(Right_Motor1, Right_Motor2, Right_Motor3, Right_Motor4);

// Intake Motors
motor HookIntake = motor(PORT1);
motor FrontIntake = motor(PORT11,true);
motor_group Intake_group = motor_group(HookIntake, FrontIntake);

// Lady Brown motors just named as Claw Motors
motor ClawMotorLeft = motor(PORT13);
motor ClawMotorRight = motor(PORT12, true);
motor_group ClawMotorGroup = motor_group(ClawMotorLeft, ClawMotorRight);

//arm motors
motor arm_Left = motor(PORT15,true);
motor arm_Right = motor(PORT17);

motor_group ArmMotorGroup = motor_group(arm_Left, arm_Right);


// Brain and LEDs
vex::brain Brain;
led ClampMotor = led(Brain.ThreeWirePort.G);
led EndGame= led(Brain.ThreeWirePort.A);
led RusharmR= led(Brain.ThreeWirePort.H);
led RusharmL= led(Brain.ThreeWirePort.F);


inertial Inertial = inertial(PORT20);
Graph graph = Graph(&Brain.Screen);

odometry Odom = odometry(odometry::odometry_pod(odometry::odometry_pod::VERTICAL, &Right_Motor1, 5.656, 0.0212), odometry::odometry_pod(), &Inertial);
chassis Drivetrain = chassis(std::bind(&odometry::getPosition, &Odom), &MotorGroupLeft, &MotorGroupRight, &Inertial, 11.3125, 0.0214);

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
bool macro_live = false;
bool EjectMode = false;
bool wasLeftPressing = false;
bool Intake = false;
bool upWasPressing = false;
bool runIntake = false;
bool downWasPressing = false;
bool intakeDirection = true;
bool ejectRingR = false;
bool ejectRingB = false;

// Optical sensor
optical Optical = optical(PORT16);
double hue = Optical.hue();
bool isRed = true;
bool intakeOnB = false;
bool intakeOnR = false;
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
//Problem could be independent task for dt
//Task on auto sort is running indefinite 


//*Eject V3.0 Red
vex::thread ejectThreadRED;
// Intake task
int intake_taskRED(){ 
    Optical.setLightPower(50, percent);
    Optical.setLight(ledState::on);
    task::sleep(100);
    Brain.Screen.print("Bug1");
    Optical.integrationTime(5);
    while (true) {
        if(intakeOnR == true);
        Brain.Screen.print("Bug9");
        Brain.Screen.clearScreen(Optical.color());
        std::cout << "\nOptical is near object? " << (Optical.isNearObject() ? "Yes" : "No") << "\n";
            if  (Optical.color() == blue ){
                
                    double target = Intake_group.position(degrees) + 695;
                    waitUntil(Intake_group.position(deg) >= target);
                    Intake_group.spin(reverse,100,percent);
                    task::sleep(250);
                    Intake_group.spin(forward,100,percent);
                        Brain.Screen.print("Bug3");
                        //ejectRingB = true;
            }
            else {  //Optical.setLightPower(5,percent);
                //Brain.Screen.print("Bug4");
            }
        if (ejectRingB) {
            //Brain.Screen.print("Bug5");
            if(Competition.isAutonomous()){
                task::sleep(610);
                //Brain.Screen.print("Bug6");
                Intake_group.stop(brake);
                //Brain.Screen.print("Bug7");
                task::sleep(50);
                //Brain.Screen.print("Bug8");
                Intake_group.spin(forward, 100, percent);
                ejectRingB = false;        
            
            }
            
        }
        task::sleep(5);
    }
}
        




//*Eject V3.0 Blue
vex::thread ejectThreadBlu;
// Intake task
int intake_taskBLU(){
    Optical.setLightPower(50, percent);
    Optical.setLight(ledState::on);
    task::sleep(100);
    Brain.Screen.print("Bug1");
    Optical.integrationTime(5);
    while (true) {
        if(intakeOnB == true);
        Brain.Screen.print("Bug9");
        Brain.Screen.clearScreen(Optical.color());
        std::cout << "\nOptical is near object? " << (Optical.isNearObject() ? "Yes" : "No") << "\n";
            if  (Optical.color() == red ){
                
                    double target = Intake_group.position(degrees) + 700;
                    waitUntil(Intake_group.position(deg) >= target);
                    Intake_group.spin(reverse,100,percent);
                    task::sleep(250);
                    Intake_group.spin(forward,100,percent);
                        Brain.Screen.print("Bug3");
            }
        }
    }
    /* Optical.setLightPower(5, percent);
    //Brain.Screen.print("Bug1");
    task::sleep(10);
    Optical.integrationTime(5);
    while (true) {
        if(intakeOnB == true);
        //Brain.Screen.print("Bug9");
            if  (Optical.color() = red){
            //Brain.Screen.print("Bug2");
                double target = Intake_group.position(degrees) + 50;
                waitUntil(Intake_group.position(deg) >= target);
                Intake_group.spin(reverse,100,percent);
                task::sleep(250);
                Intake_group.spin(forward,100,percent);
                    Brain.Screen.print("Bug3");
                    ejectRingB = true;
                }
                } else {  Optical.setLightPower(5,percent);
                    //Brain.Screen.print("Bug4");
                }
        if (ejectRingB) {
            //Brain.Screen.print("Bug5");
            if(Competition.isAutonomous()){
                task::sleep(610);
                //Brain.Screen.print("Bug6");
                Intake_group.stop(brake);
                //Brain.Screen.print("Bug7");
                task::sleep(50);
                //Brain.Screen.print("Bug8");
                Intake_group.spin(forward, 100, percent);
                ejectRingB = false;        
            
                }
            return false;}*/
        //*/
            




//Eject V2.0
/*vex::thread ejectThread;
int ringEjectPosition = 1437 / 3;
// Intake task
int intake_task() {

    Optical.setLightPower(50, percent);
    Optical.integrationTime(5);

    while (true) {
        if (Controller.ButtonUp.pressing() && !wasPressing) intakeOn = !intakeOn;
        wasPressing = Controller.ButtonUp.pressing();

        if (Controller.ButtonLeft.pressing() && !wasLeftPressing) {
            EjectMode = !EjectMode; // Toggle Eject mode
        }
        wasLeftPressing = Controller.ButtonLeft.pressing();
        if (Controller.ButtonDown.pressing()) {
            Intake_group.spin(reverse, 100, percent);
        } else {
            if (intakeOn) {
                if (Optical.isNearObject()) {
                    Optical.setLight(ledState::on);
                     if ((!isRed && isRedRing(Optical.color())) || (isRed && isBlueRing(Optical.color()))) {
                        ejectRing = true;
                    } else {
                    Optical.setLight(ledState::off);
                }
                    if (ejectRing) {
                    if (abs(((int)HookIntake.position(vex::rotationUnits::deg) % ringEjectPosition) - ringEjectPosition) < 35) {
                        if(Competition.isAutonomous()){
                            task::sleep(50);
                            task::sleep(30);
                            Intake_group.spin(reverse, 100, percent);
                            task::sleep(40);
                            Intake_group.spin(forward, 100, percent);
                            ejectRing = false;
                        }
                        task::sleep(50);
                        Intake_group.spin(reverse, 100, percent);
                        task::sleep(250);
                        ejectRing = false;
                    }
                } else {
                    Intake_group.spin(forward, 100, percent);
                }
}}}}}*/

/*Intake task
int intake_task() {

    Optical.setLightPower(50, percent);
    Optical.integrationTime(5);

    while (true) {
        if (Controller.ButtonUp.pressing() && !wasPressing) intakeOn = !intakeOn;
        wasPressing = Controller.ButtonUp.pressing();

        if (Controller.ButtonY.pressing() && !wasPressing) {
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
}*/


//------------------------------------------------------------------------------
// Pre-Autonomous Functions
//------------------------------------------------------------------------------

int autonValue = 0; // None
/**
 * @brief Function to handle pre-autonomous setup.
 */
void pre_auton(void) {
   button RedAUTONP = button(&Brain.Screen, 0,0 ,240,80, red,black,2, white, "+");
   button BluAUTONP = button(&Brain.Screen, 242,0 ,240,80, blue,black,2, white, "+");
   button RedAUTONM = button(&Brain.Screen, 0,82 ,240,80, red,black,2, white, "Mid");
   button BluAUTONM = button(&Brain.Screen, 242,82 ,240,80, blue,black,2, white, "Mid");
   button Skills = button(&Brain.Screen, 0,164 ,240,80, green,black,2, white, "SKILLS");
   button Calibrate = button(&Brain.Screen, 242,164 ,240,80, white,black,2, black, "Calibrate");

   RedAUTONP.draw();
   BluAUTONP.draw();
   RedAUTONM.draw();
   BluAUTONM.draw();
   Skills.draw();
   Calibrate.draw();


   while (true)
   {
    waitUntil(Brain.Screen.pressing());

    if (Calibrate.isPressing()) break;

    if (RedAUTONP.isPressing())
    {
        if (autonValue == 1) autonValue = 0;
        else autonValue = 1;
    }
    if (BluAUTONP.isPressing())
    {
        if (autonValue == 2) autonValue = 0;
        else autonValue = 2;
    }
    if (RedAUTONM.isPressing())
    {
        if (autonValue == 3) autonValue = 0;
        else autonValue = 3;
    }
    if (BluAUTONM.isPressing())
    {
        if (autonValue == 4) autonValue = 0;
        else autonValue = 4;
    }
    if (Skills.isPressing())
    {
        if (autonValue == 5) autonValue = 0;
        else autonValue = 5;
    }

    waitUntil(!Brain.Screen.pressing());

    switch (autonValue)
    {
        case 0:
            RedAUTONP.changeColor(red, black, white);
            BluAUTONP.changeColor(blue, black, white);
            RedAUTONM.changeColor(red, black, white);
            BluAUTONM.changeColor(blue, black, white);
            Skills.changeColor(green, black, white);
            break;
        case 1:
            RedAUTONP.changeColor(color(125, 0, 0), black, white);
            BluAUTONP.changeColor(blue, black, white);
            RedAUTONM.changeColor(red, black, white);
            BluAUTONM.changeColor(blue, black, white);
            Skills.changeColor(green, black, white);
            break;
        case 2:
            RedAUTONP.changeColor(red, black, white);
            BluAUTONP.changeColor(color(0, 0, 125), black, white);
            RedAUTONM.changeColor(red, black, white);
            BluAUTONM.changeColor(blue, black, white);
            Skills.changeColor(green, black, white);
            break;
        case 3:
            RedAUTONP.changeColor(red, black, white);
            BluAUTONP.changeColor(blue, black, white);
            RedAUTONM.changeColor(color(125, 0, 0), black, white);
            BluAUTONM.changeColor(blue, black, white);
            Skills.changeColor(green, black, white);
            break;
        case 4:
            RedAUTONP.changeColor(red, black, white);
            BluAUTONP.changeColor(blue, black, white);
            RedAUTONM.changeColor(red, black, white);
            BluAUTONM.changeColor(color(0, 0, 125), black, white);
            Skills.changeColor(green, black, white);
            break;
        case 5:
            RedAUTONP.changeColor(red, black, white);
            BluAUTONP.changeColor(blue, black, white);
            RedAUTONM.changeColor(red, black, white);
            BluAUTONM.changeColor(blue, black, white);
            Skills.changeColor(color(0, 125, 0), black, white);
            break;
    }
    
    RedAUTONP.draw();
    BluAUTONP.draw();
    RedAUTONM.draw();
    BluAUTONM.draw();
    Skills.draw();
   }

   Brain.Screen.clearScreen(red);
   task::sleep(1000);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.setPenColor(white);
    Brain.Screen.setFillColor(black);
    Brain.Screen.print("Calibrating...");

    Inertial.startCalibration();
    do {
        task::sleep(50);
    } while (Inertial.isCalibrating());

    Optical.setLightPower(50, percent);
    Optical.integrationTime(5);

    Brain.Screen.setCursor(2, 1);
    Brain.Screen.setPenColor(green);
    Brain.Screen.clearScreen(green);
    task::sleep(10);
    
    Brain.Screen.setPenColor(red);
    Brain.Screen.newLine();
    
    Inertial.setHeading(270, degrees);
    Drivetrain.setDriveConstants(0.75, 0.005, 1, 9, 0.75, 30, -12, 12, 0.5);
    Drivetrain.setTurnConstants(0.15, 0.01, 0.6, 15, 0.5, 50, -12, 12);
    Drivetrain.setSwingConstants(0.2, 0.005, 0.3, 22, 0.5, 50, -12, 12);
    Drivetrain.setArcConstants(0.25, 0.01, 0.7, 15, 0.5, 50, -12, 12);
}

//------------------------------------------------------------------------------
// Autonomous Task
//------------------------------------------------------------------------------

/**
 * @brief Function to handle autonomous control.
 */
void auton(void){
    //Inertial.setHeading(110.8, degrees);
    //Inertial.setHeading(257.5,degrees);
    Drivetrain.setDriveConstants(1.15, 0.005, 0.6 , 9, 0.75, 30, -12, 12, 0.5);
    Drivetrain.setTurnConstants(0.18, 0.01, 0.48, 15, 0.5, 50, -12, 12);
    Drivetrain.setSwingConstants(0.2, 0.005, 0.3, 22, 0.5, 50, -12, 12);
    Drivetrain.setArcConstants(0.25, 0.01, 1.2, 15, 0.5, 50, -12, 12);
    ClawMotorGroup.stop(hold);
    ArmMotorGroup.stop(hold);
    ClawMotorGroup.setVelocity(100, percent);
    ArmMotorGroup.setVelocity(100, percent);
    Intake_group.setVelocity(100, percent);
    //Drivetrain.driveFor(24);
    //Drivetrain.turnFor(90);
    //Drivetrain.swingFor(right, 90);
    //Intake_group.spin(forward, 100, percent);
    //intakeOn =true;
    task eject;

    std::cout << "\n Auton: " << autonValue << "\n";
    switch (autonValue)
    {
        case 1:
            Inertial.setHeading(110.8, degrees);
            eject = task(intake_taskRED);
            //Intake_group.spin(forward, 100, percent);
            //intakeOnR =true;
        //*Mogo Rush Red
            Drivetrain.driveFor(40);
            RusharmR.set(!RusharmR);
            task::sleep(600);
            Drivetrain.driveFor(-35,1);
            RusharmR.set(!RusharmR);
            task::sleep(750);
        //Grab Mogo Red
            Drivetrain.turnTo(295,1);
            Drivetrain.setDriveSpeed(-4.5,volt);
            task::sleep(1000); 
            ClampMotor.set(!ClampMotor);
            task::sleep(250);
            Drivetrain.driveFor(17);
            Drivetrain.turnTo(270);
            Drivetrain.driveFor(-24);
        //Score Preload Red
            ArmMotorGroup.spinFor(400, degrees);
            Intake_group.spin(forward);
            intakeOnR = true;
            Drivetrain.driveFor(3);
        //Turn towards 2,-1 Pile Red
            Drivetrain.turnTo(180);
            Drivetrain.driveFor(5);
            Drivetrain.driveFor(5);
            Drivetrain.driveFor(12, 0.25);
            Drivetrain.driveFor(12, 0.75);
            Drivetrain.driveFor(-7);
            Drivetrain.turnTo(270);
        //Drive towards -Y Wall Red
            Drivetrain.driveFor(20,5);
            Drivetrain.driveFor(10,1.5);
            Drivetrain.driveFor(10,1);
            Drivetrain.turnTo(210);
            Drivetrain.driveFor(-10);
        //Clear +,- Corner Red
            RusharmR.set(!RusharmR);
            Drivetrain.driveFor(14,1);
            task([]() -> int {
                waitUntil(Inertial.heading(deg) < 185);
                RusharmR.set(!RusharmR);
             return 0;
            });
            Drivetrain.turnTo(160);
        //Get Rings From +,- Corner Red
            Drivetrain.turnTo(210);
            Drivetrain.driveFor(8,1.5);
            task::sleep(100);
            Drivetrain.driveFor(-9,0.5);
            Drivetrain.swingFor(left, 90);
        //Puts Mogo in +,- Corner Red
            ClampMotor.set(!ClampMotor);
            Drivetrain.driveFor(-5);
            Drivetrain.driveFor(12);
            /*Drivetrain.setDriveSpeed(4.5,volt);
            task::sleep(1500); 
            Drivetrain.driveFor(-12);
            ClampMotor.set(!ClampMotor);
            Drivetrain.driveFor(-25,2);
        //*/
            break;
        case 2:
        Inertial.setHeading(257.5, degrees);
        eject = task(intake_taskBLU);
        //Intake_group.spin(forward, 100, percent);
        //intakeOnB =true;
        //*Blu Rush
            Drivetrain.driveFor(42);
            RusharmL.set(!RusharmL);
            task::sleep(600);
            Drivetrain.driveFor(-35,1);
            RusharmL.set(!RusharmL);
            task::sleep(750);
        //Grab Mogo Blu
            Drivetrain.turnTo(75);
            Drivetrain.setDriveSpeed(-4.5,volt);
            task::sleep(1000); 
            ClampMotor.set(!ClampMotor);
            Drivetrain.driveFor(17);
            Drivetrain.turnTo(90);
            Drivetrain.driveFor(-20);
        //Score Preload Blu
            ArmMotorGroup.spinFor(400, degrees);
            Intake_group.spin(forward);
            intakeOnB = true;
            Drivetrain.driveFor(3);
        //Turn towards 2,1 Pile Blu
            Drivetrain.turnTo(185);
            Drivetrain.driveFor(5);
            Drivetrain.driveFor(5);
            Drivetrain.driveFor(12, 0.25);
            Drivetrain.driveFor(12, 0.75);
            Drivetrain.driveFor(-7);
            Drivetrain.turnTo(90);
        //Drive towards +Y Wall Blu
            Drivetrain.driveFor(20,5);
            Drivetrain.driveFor(10,1.5);
            Drivetrain.driveFor(6,0.75);
            Drivetrain.turnTo(165);
            Drivetrain.driveFor(-10);
        //Clear +,+ Corner Blu
            RusharmL.set(!RusharmL);
            Drivetrain.driveFor(24,1.5);
            task([]() -> int {
                waitUntil(Inertial.heading(deg) > 145);
                    RusharmL.set(!RusharmL);
                    return 0;
            });
            Drivetrain.turnTo(270);
        //Get Rings From +,- Corner Blu
            Drivetrain.turnTo(140);
            Drivetrain.driveFor(8,1.5);
            task::sleep(100);
            Drivetrain.driveFor(-9,0.5);
            Drivetrain.swingFor(right, 120);
        //Puts Mogo in +,- Corner Blu
            Drivetrain.setDriveSpeed(4.5,volt);
            task::sleep(1500); 
            Drivetrain.driveFor(-12);
            ClampMotor.set(!ClampMotor);
            Drivetrain.driveFor(-25,2);
            Drivetrain.driveFor(10);
        //*/
            break;
        case 5:
        /*Inertial.setHeading(110.8, degrees);
            eject = task(intake_taskBLU);
            //Intake_group.spin(forward, 100, percent);
            //intakeOnR =true;
        //*Mogo Rush Red
            Drivetrain.driveFor(40);
            RusharmR.set(!RusharmR);
            task::sleep(600);
            Drivetrain.driveFor(-35,1);
            RusharmR.set(!RusharmR);
            task::sleep(750);
        //Grab Mogo Red
            Drivetrain.turnTo(295,1);
            Drivetrain.setDriveSpeed(-4.5,volt);
            task::sleep(1000); 
            ClampMotor.set(!ClampMotor);
            task::sleep(250);
            Drivetrain.driveFor(17);
            Drivetrain.turnTo(270);
            Drivetrain.driveFor(-24);
        //Score Preload Red
            ArmMotorGroup.spinFor(400, degrees);
            Intake_group.spin(forward);
            intakeOnB = true;
            Drivetrain.driveFor(3);
        //Turn towards 2,-1 Pile Red
            Drivetrain.turnTo(180);
            Drivetrain.driveFor(5);
            Drivetrain.driveFor(5);
            Drivetrain.driveFor(12, 0.25);
            Drivetrain.driveFor(12, 0.75);
            Drivetrain.driveFor(-7);
            Drivetrain.turnTo(270);
        //Drive towards -Y Wall Red
            Drivetrain.driveFor(20,5);
            Drivetrain.driveFor(10,1.5);
            Drivetrain.driveFor(10,1);
            Drivetrain.turnTo(210);
            Drivetrain.driveFor(-10);
        //Clear +,- Corner Red
            RusharmR.set(!RusharmR);
            Drivetrain.driveFor(14,1);
            task([]() -> int {
                waitUntil(Inertial.heading(deg) < 185);
                RusharmR.set(!RusharmR);
             return 0;
            });
            Drivetrain.turnTo(160);
        //Get Rings From +,- Corner Red
            Drivetrain.turnTo(210);
            Drivetrain.driveFor(8,1.5);
            task::sleep(100);
            Drivetrain.driveFor(-9,0.5);
            Drivetrain.swingFor(left, 90);
            Drivetrain.driveFor(30);
            Drivetrain.turnTo(150);*/
        Inertial.setHeading(90, degrees);
        ClawMotorGroup.stop(hold);
        ArmMotorGroup.stop(hold);
        ClawMotorGroup.setVelocity(100, percent);
        ArmMotorGroup.setVelocity(100, percent);
        //first ring
            Intake_group.spin(forward, 100, percent);
            Drivetrain.driveFor(33,2);
            task::sleep(500);
            Intake_group.stop(brake);
            Drivetrain.turnTo(0);
            task::sleep(250);
            Drivetrain.driveFor(-15,1.5);
            Drivetrain.setDriveSpeed(-4.5,volt);
            task::sleep(500); 
        //grabs mogo
            ClampMotor.set(!ClampMotor);
            task::sleep(150);
            Intake_group.spin(forward, 100, percent);
            task::sleep(750);
            Drivetrain.driveFor(-5,0.25);
        //Back up from mogo 
            Drivetrain.turnTo(105);
            Drivetrain.driveFor(14);
            Drivetrain.turnTo(0);
            Drivetrain.driveFor(10);
            Drivetrain.turnTo(270);
            Drivetrain.driveFor(45);
            task::sleep(1000);
        //Turn towards -,- cornner
            Drivetrain.turnTo(215);
            Drivetrain.driveFor(20,0.75);
            Drivetrain.driveFor(-10);
            Drivetrain.turnTo(45);
            Intake_group.stop(brake);
            ClampMotor.set(!ClampMotor);
            Drivetrain.driveFor(-30,0.5);
        //Wall Reset
            Drivetrain.driveFor(12);
            Drivetrain.turnTo(90);
            Drivetrain.setDriveSpeed(-4,volt);
            task::sleep(1100);
            Drivetrain.driveFor(48);
            task::sleep(0.5);
            Drivetrain.turnTo(0);
            Drivetrain.setDriveSpeed(-4,volt);
            task::sleep(1100);
            Inertial.setHeading(0,degrees);
            Drivetrain.driveFor(15);
            task::sleep(0.5);
        //Blue Side Start
            Drivetrain.turnTo(90);
            Intake_group.spin(forward,100,percent);
            Drivetrain.driveFor(41,2.5);
            task::sleep(500);
            Intake_group.stop(brake);
            Drivetrain.turnTo(180);
            Drivetrain.setDriveSpeed(-4.5,volt);
            task::sleep(1000);
            Drivetrain.stopDrive(brake);
            ClampMotor.set(!ClampMotor);
            Intake_group.spin(forward,100,percent);
            task::sleep(1000);
            Intake_group.stop(coast);
        //Turn towards mid
            Drivetrain.turnTo(315);
            Drivetrain.setDriveSpeed(3,volt);
            task::sleep(2000);
            Intake_group.spin(forward,100,percent);
            Drivetrain.driveFor(2);
            Drivetrain.turnTo(225);
            Drivetrain.driveFor(4);
            task::sleep(750);
            Drivetrain.turnTo(315);
            Drivetrain.driveFor(15);
            Drivetrain.driveFor(-15);
            task::sleep(750);
            Drivetrain.turnTo(45);
            Drivetrain.driveFor(2);
            task::sleep(750);
            Drivetrain.turnTo(270);
            Drivetrain.driveFor(3);
            Drivetrain.driveFor(-3);
            task::sleep(750);
            Drivetrain.turnTo(0);
            Drivetrain.driveFor(3);
            Drivetrain.driveFor(-3);
            task::sleep(750);
            Drivetrain.turnTo(135);
            Intake_group.stop(coast);



        //*/
            break;
        case 4:
            Inertial.setHeading(257.5, degrees);
            eject = task(intake_taskBLU);
            //Blu Mid Rush
            Drivetrain.setArcSpeed(right, 18, 12, volt);
            waitUntil(Inertial.heading(deg) > 315 - 10);
            Drivetrain.driveFor(12);
            RusharmR.set(!RusharmR);
            task::sleep(500);
            Drivetrain.driveFor(-24);
            RusharmR.set(!RusharmR);
            Drivetrain.turnTo(140);
            Drivetrain.driveFor(-18);
            Drivetrain.driveFor(-6);
            ClampMotor.set(!ClampMotor);
            Drivetrain.driveFor(24);
            ArmMotorGroup.spinFor(400, degrees);
            Intake_group.spin(forward);
            intakeOnB = true;
            Drivetrain.turnTo(135);
            Drivetrain.driveFor(-12);
            Drivetrain.turnTo(185);
            Drivetrain.driveFor(36, 1);
            Drivetrain.driveFor(-7);
            Drivetrain.turnTo(90);
        //Drive towards +Y Wall Blu
            Drivetrain.driveFor(20,5);
            Drivetrain.driveFor(10,1.5);
            Drivetrain.driveFor(6,0.75);
            Drivetrain.turnTo(165);
            Drivetrain.driveFor(-10);
        //Clear +,+ Corner Blu
            RusharmL.set(!RusharmL);
            Drivetrain.driveFor(24,1.5);
            task([]() -> int {
                waitUntil(Inertial.heading(deg) > 145);
                    RusharmL.set(!RusharmL);
                    return 0;
            });
            Drivetrain.turnTo(270);
        //Get Rings From +,- Corner Blu
            Drivetrain.turnTo(140);
            Drivetrain.driveFor(8,1.5);
            task::sleep(100);
            Drivetrain.driveFor(-9,0.5);
            Drivetrain.swingFor(right, 120);
        //Puts Mogo in +,- Corner Blu
            Drivetrain.setDriveSpeed(4.5,volt);
            task::sleep(1500); 
            Drivetrain.driveFor(-12);
            ClampMotor.set(!ClampMotor);
            Drivetrain.driveFor(-25,2);
            Drivetrain.driveFor(10);
            break;
        case 3:
        Inertial.setHeading(110.8, degrees);
        eject = task(intake_taskRED);
        //Red Mid Rush
        Drivetrain.setArcSpeed(left, 18, 12, volt);
        waitUntil(Inertial.heading(deg) < 45 + 10);
        Drivetrain.driveFor(10);
        RusharmL.set(!RusharmL);
        task::sleep(500);
        Drivetrain.driveFor(-24);
        RusharmL.set(!RusharmL);
        Drivetrain.turnTo(-140);
        Drivetrain.driveFor(-18);
        Drivetrain.driveFor(-6);
        ClampMotor.set(!ClampMotor);
        Drivetrain.driveFor(12);
        ArmMotorGroup.spinFor(400, degrees);
        Intake_group.spin(forward);
        intakeOnR = true;
        Drivetrain.turnTo(180);
        Drivetrain.driveFor(36);
        task::sleep(750);
        Drivetrain.driveFor(-12);
        Drivetrain.turnTo(270);
        //Drive towards -Y Wall Red
            Drivetrain.driveFor(20,5);
            Drivetrain.driveFor(10,1.5);
            Drivetrain.driveFor(10,1);
            Drivetrain.turnTo(210);
            Drivetrain.driveFor(-10);
        //Clear +,- Corner Red
            RusharmR.set(!RusharmR);
            Drivetrain.driveFor(14,1);
            task([]() -> int {
                waitUntil(Inertial.heading(deg) < 175);
                RusharmR.set(!RusharmR);
             return 0;
            });
            Drivetrain.turnTo(150);
        //Get Rings From +,- Corner Red
            Drivetrain.turnTo(210);
            Drivetrain.driveFor(8,1.5);
            task::sleep(100);
            Drivetrain.driveFor(-9,0.5);
            Drivetrain.swingFor(left, Inertial.heading(deg) - 100, 1.25);
            Drivetrain.setDriveSpeed(4.5, volt);
            task::sleep(1500);
        //Puts Mogo in +,- Corner Red
        Drivetrain.turnTo(80, 1);
            ClampMotor.set(!ClampMotor);
            Drivetrain.driveFor(-45,1.5);
            Drivetrain.driveFor(12,1);
            break;
    }
    /*Blu Rush
    Drivetrain.driveFor(52);
    RusharmL.set(!RusharmL);
    wait(0.60, seconds);
    Drivetrain.driveFor(-35,1);
    RusharmL.set(!RusharmL);
    wait(0.75,sec);
    //Grab Mogo Blu
    Drivetrain.turnTo(75);
    Drivetrain.driveFor(-15);
    Drivetrain.driveFor(-10);
    ClampMotor.set(!ClampMotor);
    Drivetrain.driveFor(17);
    Drivetrain.turnTo(90);
    Drivetrain.driveFor(-20);
    //Score Preload Blu
    ArmMotorGroup.spinFor(400, degrees);
    Intake_group.spin(forward);
    intakeOn = true;
    Drivetrain.driveFor(3);
    //Turn towards 2,1 Pile Blu
    Drivetrain.turnTo(185);
    Drivetrain.driveFor(5);
    Drivetrain.driveFor(5);
    Drivetrain.driveFor(12, 0.25);
    Drivetrain.driveFor(12, 0.75);
    Drivetrain.driveFor(-7);
    Drivetrain.turnTo(90);
    //Drive towards +Y Wall Blu
    Drivetrain.driveFor(20,5);
    Drivetrain.driveFor(10,1.5);
    Drivetrain.driveFor(6,0.75);
    Drivetrain.turnTo(165);
    Drivetrain.driveFor(-10);
    //Clear +,+ Corner Blu
    RusharmL.set(!RusharmL);
    Drivetrain.driveFor(24,1.5);
    task([]() -> int {
        waitUntil(Inertial.heading(deg) > 145);
        RusharmL.set(!RusharmL);
        return 0;
    });
    Drivetrain.turnTo(270);
    //Get Rings From +,- Corner Blu
    Drivetrain.turnTo(140);
    Drivetrain.driveFor(8,1.5);
    wait(100,msec);
    Drivetrain.driveFor(-9,0.5);
    Drivetrain.swingFor(right, 120);
    //Puts Mogo in +,- Corner Blu
    Drivetrain.setDriveSpeed(4.5,volt);
    task::sleep(1500); 
    Drivetrain.driveFor(-12);
    ClampMotor.set(!ClampMotor);
    Drivetrain.driveFor(-25,2);
//*/


    /*Mid Rush
    Drivetrain.driveFor(45);
    RusharmL.set(!RusharmL);
    wait(0.60, seconds);
    Drivetrain.driveFor(-35,1);*/
    
    
    /*Mogo Rush Red
    Drivetrain.driveFor(53);
    RusharmR.set(!RusharmR);
    wait(0.60, seconds);
    Drivetrain.driveFor(-35,1);
    RusharmR.set(!RusharmR);
    wait(0.75,sec);
    //Grab Mogo Red
    Drivetrain.turnTo(285);
    Drivetrain.driveFor(-15);
    Drivetrain.driveFor(-10);
    ClampMotor.set(!ClampMotor);
    Drivetrain.driveFor(17);
    Drivetrain.turnTo(270);
    Drivetrain.driveFor(-20);
    //Score Preload Red
    ArmMotorGroup.spinFor(400, degrees);
    Intake_group.spin(forward);
    intakeOn = true;
    Drivetrain.driveFor(3);
    //Turn towards 2,-1 Pile Red
    Drivetrain.turnTo(180);
    Drivetrain.driveFor(5);
    Drivetrain.driveFor(5);
    Drivetrain.driveFor(12, 0.25);
    Drivetrain.driveFor(12, 0.75);
    Drivetrain.driveFor(-7);
    Drivetrain.turnTo(270);
    //Drive towards -Y Wall Red
    Drivetrain.driveFor(20,5);
    Drivetrain.driveFor(10,1.5);
    Drivetrain.driveFor(6,0.75);
    Drivetrain.turnTo(195);
    Drivetrain.driveFor(-10);
    //Clear +,- Corner Red
    RusharmR.set(!RusharmR);
    Drivetrain.driveFor(14,1);
    task([]() -> int {
        waitUntil(Inertial.heading(deg) > 145);
        RusharmR.set(!RusharmR);
        return 0;
    });
    Drivetrain.turnTo(90);
    //Get Rings From +,- Corner Red
    Drivetrain.turnTo(210);
    Drivetrain.driveFor(8,1.5);
    wait(100,msec);
    Drivetrain.driveFor(-9,0.5);
    Drivetrain.swingFor(left, 120);
    //Puts Mogo in +,- Corner Red
    Drivetrain.setDriveSpeed(4.5,volt);
    task::sleep(1500); 
    Drivetrain.driveFor(-12);
    ClampMotor.set(!ClampMotor);
    Drivetrain.driveFor(-25,2);
    //*/



}



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
    ClawMotorGroup.setVelocity(100, percent);
    ArmMotorGroup.setVelocity(100, percent);
    Intake_group.spin(forward, 50, percent);
    //first ring
    Drivetrain.driveFor(29,2);
    task::sleep(150);
    Intake_group.stop(coast);
    Drivetrain.turnFor(-85);
    task::sleep(250);
    Drivetrain.driveFor(-15,2);
    Drivetrain.driveFor(-5);
    task::sleep(250);
    //grabs mogo
    ClampMotor.set(!ClampMotor);
    task::sleep(150);
    Intake_group.spin(forward, 100, percent);
    task::sleep(750);
    Drivetrain.driveFor(25);
    //turns to the middle of the feild
    Drivetrain.turnFor(229);
    //inakes middle ring
    Drivetrain.driveFor(50,3);
    Drivetrain.driveFor(-5);
    Drivetrain.turnFor(180);
    ClampMotor.set(!ClampMotor);
    Drivetrain.driveFor(-5);
   

}

void match(void) {
    Inertial.setHeading(220, degrees);    



}
void autonomous(void) {
    
    //match();
    //skills();
    auton();

    



    


}

//------------------------------------------------------------------------------
// User Control Task
//------------------------------------------------------------------------------

/**
 * @brief Function to handle user control.
 */
void usercontrol(void) {
    //vex::thread intake_Functionality = vex::thread(intake_task);
    // while (macro_live = false){
    //         Inertial.setHeading(270, degrees);
    //         Drivetrain.setDriveConstants(1.15, 0.005, 0.6 , 9, 0.75, 30, -12, 12, 0.5);
    //         Drivetrain.setTurnConstants(0.18, 0.01, 0.48, 15, 0.5, 50, -12, 12);
    //         Drivetrain.setSwingConstants(0.2, 0.005, 0.3, 22, 0.5, 50, -12, 12);
    //         Drivetrain.setArcConstants(0.25, 0.01, 0.7, 15, 0.5, 50, -12, 12);
        
        
    //         //Drivetrain.driveFor(24);
    //         //Drivetrain.turnFor(90);
    //         //Drivetrain.swingFor(right,90);
    //         ClawMotorGroup.stop(hold);
    //         ArmMotorGroup.stop(hold);
    //         ClawMotorGroup.setVelocity(100, percent);
    //         ArmMotorGroup.setVelocity(100, percent);
    //         Intake_group.spin(forward, 50, percent);
    //         //first ring
    //         Drivetrain.driveFor(32,2);
    //         wait(0.15,seconds);
    //         Intake_group.stop(coast);
    //         Drivetrain.turnFor(-77);
    //         wait(.25,seconds);
    //         Drivetrain.driveFor(-17);
    //         Drivetrain.driveFor(-8);
    //         wait(.25, seconds);
    //         //grabs mogo
    //         ClampMotor.set(!ClampMotor);
    //         wait(0.15, seconds);
    //         Intake_group.spin(forward, 100, percent);
    //         wait(0.75, seconds);
    //         Drivetrain.driveFor(22);
    //         //turns to the middle of the feild
    //         Drivetrain.turnFor(47);
    //         Intake_group.stop(coast);
    //         //inakes middle ring
    //         Drivetrain.driveFor(13);
    //         Drivetrain.driveFor(6);
    //         Intake_group.spin(forward, 100, percent);
    //         Drivetrain.driveFor(5);
    //         wait(0.25, seconds);
    //         Intake_group.stop(coast);
    //         Drivetrain.driveFor(-30);
    //         Intake_group.spinFor(reverse, 0.5, seconds);
    //         wait(0.25, seconds);
    //         //ArmMotorGroup.spinFor(561, degrees);
    //         //ClawMotorGroup.spinFor(-160, degrees);
    //         Intake_group.spin(forward,100,percent);
    //         wait(0.5, seconds);
    //         Drivetrain.turnFor(135);
    //         ArmMotorGroup.spinFor(200, degrees);
    //         //BRUTILLY RAMS INTO THE WALL TO RESET
    //         Drivetrain.driveFor(10,2);
    //         Drivetrain.driveFor(30,2);
    //         wait(0.5, seconds);
    //         Drivetrain.driveFor(-3);
    //         wait(0.5, seconds);
    //         Intake_group.stop(coast);
    //         Drivetrain.turnFor(-90, 1);
    //         Intake_group.spin(forward, 100, percent);
    //         Drivetrain.driveFor(25);
    //         //Swings grabs two red rings and runs for the red ring by the corner
    //         Drivetrain.swingFor(left, 180);
    //         Intake_group.spin(forward, 100, percent);
    //         Drivetrain.driveFor(36,2);
    //         //BRUTILLY RAMS INTO THE WALL TO RESET
    //         Drivetrain.driveFor(36,1);
    //         wait(0.75, seconds);
    //         Drivetrain.driveFor(-10);
    //         //Turns to the corner
    //         Drivetrain.turnFor(-45, 1);
    //         //Intakes the corener ring stops intake then moves the arms to where it can take the red ring into the claw
    //         ClawMotorGroup.setVelocity(50, percent);
    //         ArmMotorGroup.setVelocity(50, percent);
    //         Drivetrain.driveFor(20, 1);
    //         Intake_group.stop(coast);
    //         ArmMotorGroup.spinFor(361, degrees);
    //         ClawMotorGroup.spinFor(-160, degrees);
    //         Intake_group.spin(forward, 100, percent);
    //         //backs up then spins 180 to dump the mogo into the corner 
    //         ClawMotorGroup.setVelocity(100, percent);
    //         ArmMotorGroup.setVelocity(100, percent);
    //         Drivetrain.driveFor(-15, 1);
    //         Drivetrain.turnFor(-180, 1);
    //         wait(.5, seconds);
    //         ClampMotor.on();
    //         wait(.5, seconds);
    //         Drivetrain.driveFor(-25, 1);
    //         //Drives back and resets of the wall again 
    //         Drivetrain.driveFor(32, 1);
    //         Drivetrain.turnFor(45, 1);
    //         Intake_group.stop(coast);
    //         //BRUTIAL WALL RAM
    //         wait(1, seconds);
    //         Drivetrain.driveFor(-45, 1.5);
    //         Drivetrain.driveFor(48, 1);
    //         Drivetrain.turnFor(90, 1);
    //         //BRUTIAL WALL RAM
    //         Drivetrain.driveFor(45, 0.75);
    //         wait(1, seconds);
    //         Drivetrain.driveFor(-18, 1);
    //         Drivetrain.turnFor(-90, 1);
    //         Drivetrain.driveFor(22.75, 1.5);
    //         Drivetrain.turnFor(90, 1);
    //         //Ready to score wall stake
    //         ArmMotorGroup.spinFor(-200, degrees);
    //         Drivetrain.driveFor(24,1.5);
    //         ClawMotorGroup.spinFor(-1200, degrees);
    //         Drivetrain.driveFor(-24, 1.5);
    //         //AUTON PART 2 (second half of the feild)
    //         Drivetrain.turnFor(-90, 1.5);
    //     }
    //     if (Controller.ButtonA.pressing()){
    //         macro_live = true;
    //     }

    while (true) {
        // Toggle Clamp LED on ButtonL1 Press
        if (Controller.ButtonL1.pressing() && !L1WasPressing) {
            ClampMotor.set(!ClampMotor);
        }

        // Toggle RatchetMotor LED on ButtonY Press
        if(Controller.ButtonY.PRESSED){
            RusharmR.set(!RusharmR);
        }
        if(Controller.ButtonRight.PRESSED){
            RusharmL.set(!RusharmL);
        }
        if (Controller.ButtonB.pressing()) {
            ClawMotorGroup.spin(reverse, 100, percent);
        }else if (Controller.ButtonX.pressing()) {
            ClawMotorGroup.spin(forward, 100, percent);
            intakeOnR = true;
        }
        else{
          ClawMotorGroup.stop(brake);
        }

        if (Controller.ButtonR1.pressing()) {
            ArmMotorGroup.spin(fwd, 100, percent);
        }
        else if (Controller.ButtonR2.pressing()) {
            ArmMotorGroup.spin(reverse, 100, percent);
        }
        else{
            ArmMotorGroup.stop(brake);
        }
        
        if (Controller.ButtonUp.pressing() && !upWasPressing){
            if (runIntake)
            {
                if (intakeDirection) runIntake = false;
            }
            else {
                runIntake = true;
            }
            intakeDirection = true;
        }   
        if (Controller.ButtonDown.pressing() && !downWasPressing){
            if (runIntake)
            {
                if (!intakeDirection) runIntake = false;
            }
            else {
                runIntake = true;
            }
            intakeDirection = false;
        }      
        if (runIntake) {
            if (intakeDirection){
                Intake_group.spin(forward, 100, percent);
            } 
            else {
                Intake_group.spin(reverse, 100, percent);
            }
        }
        else{
            Intake_group.stop(brake);
        }


        if (Controller.ButtonRight.pressing() && !RightwasPressing) {
            EndGame.set(!EndGame);
        }
        // Update Toggle States
        RightwasPressing = Controller.ButtonRight.pressing();
        L1WasPressing = Controller.ButtonL1.pressing();
        YwasPressing = Controller.ButtonY.pressing();
        upWasPressing = Controller.ButtonUp.pressing();
        downWasPressing = Controller.ButtonDown.pressing();

        // Set Drivetrain control
        MotorGroupLeft.spin(fwd, (Controller.Axis3.position() + Controller.Axis1.position())*.12, volt);
        MotorGroupRight.spin(fwd,( Controller.Axis3.position() - Controller.Axis1.position())*0.12 , volt);
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
