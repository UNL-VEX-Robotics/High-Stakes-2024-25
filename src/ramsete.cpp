/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       purepursuit.cpp                                           */
/*    Author:       Bryce Closman - UNLVEXU                                   */
/*    Created:      09/16/2024                                                */
/*    Description:  Pure Pursuit Source Code                                  */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "ramsete.h"


/**
 * Private function that calculates the distance between 2 points
 * 
 * @param   p0  the first point
 * @param   p1  the second point
 * 
 * @return  the distance between p0 and p1
 */
float ramsete::getDistance(std::vector<float> p0, std::vector<float> p1){
    return hypotf(p0.at(0) - p1.at(0), p0.at(1) - p1.at(1));
}


/**
 * Private function that converts degrees to radians
 * 
 * @param   deg degrees
 * 
 * @return  radians
 */
float ramsete::toRad(float deg){
    return deg * (M_PI / 180);
}


/**
 * Private function that restrains a number to a range
 * 
 * @param   num the number to be restrained
 * @param   min the minimum acceptable value
 * @param   max the maximum acceptable value
 * 
 * @return the restrained number
 */
float ramsete::restrain(float num, float min, float max){
    while(num > max) num -= (max - min);
    while(num < min) num += (max - min);
    return num;
}


/**
 * Private function that determines the target point for the controller
 * 
 * @param   robotPosition   the robot's position
 * @param   Path            the path the robot is following
 * @param   currentTarget   the current target represented as an int
 * 
 * @return  the new target point
 */
int ramsete::getTargetPoint(std::vector<float> robotPosition, std::vector<pathPlanner::Point> Path, int currentTarget){
    int target = currentTarget;
    
    for(int i = target; i < Path.size(); i++){
        if(this->getDistance(robotPosition, {Path.at(target).x, Path.at(target).y}) <= this->tolerance) target++;
        else break;
    }

    return target;
}


/**
 * Private function that calculates the local error
 * 
 * @param   robotPosition   the position of the robot (x, y, heading(deg))
 * @param   targetPoint     the target point
 * 
 * @return  the local error (x, y, heading)
 */
std::vector<float> ramsete::getLocalError(std::vector<float> robotPosition, pathPlanner::Point targetPoint){
    float robotHeading = this->toRad(robotPosition.at(2));

    float xError = targetPoint.x - robotPosition.at(0);
    float yError = targetPoint.y - robotPosition.at(1);
    float headingError = this->restrain(targetPoint.heading - robotHeading, -M_PI, M_PI);

    float localXError = cosf(robotHeading) * xError + sinf(robotHeading) * yError;
    float localYError = -sinf(robotHeading) * xError + cosf(robotHeading) * yError;
    
    return {localXError, localYError, headingError};
}


/**
 * Function that runs through a path using the ramsete controller
 * 
 * @param   Path    the path for the robot to follow
 */
void ramsete::runPath(std::vector<pathPlanner::Point> Path){

    this->Path = Path;

    std::vector<float> robotPosition = this->getRobotPosition();
    this->currentPoint = this->getTargetPoint(robotPosition, Path, 0);
    
    while(this->currentPoint != Path.size()){
        std::vector<float> error = this->getLocalError(robotPosition, Path.at(this->currentPoint));

        float desiredVelocity = Path.at(this->currentPoint).velocity;
        float desiredAngularVelocity = Path.at(this->currentPoint).angularVelocity;

        if(desiredAngularVelocity == 0 && desiredVelocity == 0) {
            desiredVelocity = 0.01 * error.at(0);
            desiredAngularVelocity = 0.01 * error.at(2);
        }
        float gain = 2 * zeta * sqrtf(powf(desiredAngularVelocity, 2) + beta * powf(desiredVelocity, 2));

        float velocity = desiredVelocity * cosf(error.at(2)) + gain * error.at(0);
        float angularVelocity;

        if(error.at(2) != 0) angularVelocity = desiredAngularVelocity + gain * error.at(0) + (beta * desiredVelocity * sinf(error.at(2)) * error.at(1)) / error.at(2);
        else angularVelocity = desiredAngularVelocity + gain * error.at(0);

        float radius = velocity / angularVelocity;
        float leftVelocity = angularVelocity * (radius + this->trackWidth / 2);
        float rightVelocity = angularVelocity * (radius - this->trackWidth / 2);

        float leftMotorRPM = (leftVelocity * 60) / (this->gearRatio * M_PI * this->wheelDiameter);
        float rightMotorRPM = (rightVelocity * 60) / (this->gearRatio * M_PI * this->wheelDiameter);

        this->Left->spin(vex::directionType::fwd, leftMotorRPM, vex::velocityUnits::rpm);
        this->Right->spin(vex::directionType::fwd, rightMotorRPM, vex::velocityUnits::rpm);

        vex::this_thread::sleep_for(10);

        robotPosition = this->getRobotPosition();
        this->currentPoint = this->getTargetPoint(robotPosition, Path, this->currentPoint);
    }

    Left->stop(vex::brakeType::hold);
    Right->stop(vex::brakeType::hold);
}


/**
 * Function to set the constants of the ramsete controller
 * 
 * @param   beta    beta constant (p constant)
 * @param   zeta    zeta constant (d constant)
 */
void ramsete::setConstants(float beta, float zeta){
    this->beta = beta;
    this->zeta = zeta;
}


/**
 * Function to get the current target point
 * 
 * @return current target point
 */
pathPlanner::Point ramsete::getCurrentTarget(){
    return this->Path.at(this->currentPoint);
}


/**
 * Constructor method for the ramsete controller
 * 
 * @param   getRobotPosition    function returning the robot's position as a vector (x, y, heading): std::bind(&odom::getPosition, &myOdom)
 * @param   Left                pointer to the left motor group of the drivetrain
 * @param   Right               pointer to the right motor group of the drivetrain
 * @param   trackWidth          the physical track width of the robot
 * @param   wheelDiameter       the diameter of the wheels used on the drivetrain
 * @param   gearRatio           the ratio from the motor to the wheels on the drivetrain (wheel rpm / motor rpm)
 * @param   tolerance           the tolerance before the controller continues to the next point
 */
ramsete::ramsete(std::function<std::vector<float>()> getRobotPosition, vex::motor_group* Left, vex::motor_group* Right, float trackWidth, float wheelDiameter, float gearRatio, float tolerance){
    this->getRobotPosition = getRobotPosition;
    this->Left = Left;
    this->Right = Right;
    this->trackWidth = trackWidth;
    this->wheelDiameter = wheelDiameter;
    this->gearRatio = gearRatio;
    this->tolerance = tolerance;
}