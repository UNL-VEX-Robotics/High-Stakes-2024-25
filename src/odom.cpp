/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       odom.cpp                                                  */
/*    Author:       Bryce Closman - UNLVEXU                                   */
/*    Created:      08/09/2024                                                */
/*    Description:  Odometry Class source code                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "odom.h"

/**
 * Private function that converts degrees to radians
 * 
 * @param   deg Angle measure in degrees
 * 
 * @return  Angle measure in radians
 */
float odom::degToRad(float deg){
    return deg * (M_PI / 180);
}

/**
 * Starts and contains the odometry loop
 * Updates at a constant rate defined by user
 * Based on the 5225 Pilons odometry: http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf
 */
void odom::start(){
    this->isRunning = true;

    float previousVertical;
    float previousHorizontal;
    if(this->usesRotation){
        previousVertical = this->verticalRotation->position(vex::rotationUnits::deg) \
            * this->verticalInchesPerDegree;
        previousHorizontal = this->horizontalRotation->position(vex::rotationUnits::deg) \
            * this->horizontalInhcesPerDegree;
    }
    else{
        previousVertical = this->verticalEncoder->position(vex::rotationUnits::deg) \
            * this->verticalInchesPerDegree;
        previousHorizontal = this->horizontalEncoder->position(vex::rotationUnits::deg) \
            * this->horizontalInhcesPerDegree;
    }
    float previousHeading = odom::degToRad(this->Inertial->rotation(vex::rotationUnits::deg));

    while(isRunning){
        auto cycleStart = std::chrono::system_clock::now();

        float verticalPosition;
        float horizontalPosition;
        if(this->usesRotation){
            verticalPosition = this->verticalRotation->position(vex::rotationUnits::deg) \
                * this->verticalInchesPerDegree;
            horizontalPosition = this->horizontalRotation->position(vex::rotationUnits::deg) \
                * this->horizontalInhcesPerDegree;
        }
        else{
            verticalPosition = this->verticalEncoder->position(vex::rotationUnits::deg) \
                * this->verticalInchesPerDegree;
            horizontalPosition = this->horizontalEncoder->position(vex::rotationUnits::deg) \
                * this->horizontalInhcesPerDegree;
        }
        float heading = odom::degToRad(this->Inertial->rotation(vex::rotationUnits::deg));
        this->robotPosition.at(2) = this->Inertial->heading(vex::rotationUnits::deg);

        float changeInVertical = verticalPosition - previousVertical;
        float changeInHorizontal = horizontalPosition - previousHorizontal;
        float changeInHeading = heading - previousHeading;

        float localX;
        float localY;
        if(changeInHeading == 0){
            localX = changeInHorizontal;
            localY = changeInVertical;
        }
        else{
            localX = (2 * sinf(changeInHeading / 2)) \
                * ((changeInHorizontal / changeInHeading) + horizontalDistanceFromCenter);
            localY = (2 * sinf(changeInHeading / 2)) \
                * ((changeInVertical / changeInHeading) + verticalDistanceFromCenter);
        }

        float localPolarAngle;
        float polarRadius;
        if(localX == 0 && localY == 0){
            localPolarAngle = 0;
            polarRadius = 0;
        }
        else{
            localPolarAngle = atan2f(localY, localX);
            polarRadius = sqrtf(powf(localX, 2) + powf(localY, 2));
        }

        float globalPolarAngle = localPolarAngle - previousHeading - (changeInHeading / 2);

        previousVertical = verticalPosition;
        previousHorizontal = horizontalPosition;
        previousHeading = heading;

        float changeinX = polarRadius * cosf(globalPolarAngle);
        float changeinY = polarRadius * sinf(globalPolarAngle);

        this->robotPosition.at(0) += changeinX;
        this->robotPosition.at(1) += changeinY;

        auto timeElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - cycleStart).count();

        vex::this_thread::sleep_for(this->updateRateMilliseconds - timeElapsed);
    }
}

/**
 * Stops the odometry loop
 */
void odom::stop(){
    this->isRunning = false;
}

/**
 * Constructor method using the vex V5 Rotation Sensors
 * Creates an odom object
 * 
 * @param   verticaRotation                 The V5 Rotation Sensor that is on the vertical tracking wheel
 * @param   horizontalRotation              The V5 Rotation Sensor that is on the horizontal tracking wheel
 * @param   Inertial                        The V5 Inertial sensor that is on the robot
 * @param   verticalDistanceFromCenter      The physical distance from the tracking center to the vertical tracking wheel, in inches
 * @param   verticalIncherPerDegree         The number of inches per degree of rotation of the vertical tracking wheel
 * @param   horizontalDistanceFromCenter    The physical distance from the tracking center to the horizontal tracking wheel, in inches
 * @param   horizontalInchesPerDegree       The number of inches per degree of rotation of the horizontal tracking wheel
 * @param   updateRateMilliseconds          The desired time between cycles of the odometry loop, in milliseconds, generally 5 or 10
 */
odom::odom(vex::rotation &verticalRotation, vex::rotation &horizontalRotation, vex::inertial &Inertial, \
    float verticalDistanceFromCenter, float verticalInchesPerDegree, float horizontalDistanceFromCenter, \
        float horizontalInchesPerDegree, int updateRateMilliseconds){

    this->verticalRotation = &verticalRotation;
    this->horizontalRotation = &horizontalRotation;
    this->Inertial = &Inertial;
    this->verticalDistanceFromCenter = verticalDistanceFromCenter;
    this->verticalInchesPerDegree = verticalInchesPerDegree;
    this->horizontalDistanceFromCenter = horizontalDistanceFromCenter;
    this->horizontalInhcesPerDegree = horizontalInchesPerDegree;
    this->updateRateMilliseconds = updateRateMilliseconds;
    this->usesRotation = true;
}

/**
 * Constructor method using the 3-Wire VEX Encoders
 * Creates an odom object
 * 
 * @param   verticaEncoder                  The 3-Wire VEX Encoder that is on the vertical tracking wheel
 * @param   horizontalEncoder               The 3-wire VEX Encoder that is on the horizontal tracking wheel
 * @param   Inertial                        The V5 Inertial sensor that is on the robot
 * @param   verticalDistanceFromCenter      The physical distance from the tracking center to the vertical tracking wheel, in inches
 * @param   verticalIncherPerDegree         The number of inches per degree of rotation of the vertical tracking wheel
 * @param   horizontalDistanceFromCenter    The physical distance from the tracking center to the horizontal tracking wheel, in inches
 * @param   horizontalInchesPerDegree       The number of inches per degree of rotation of the horizontal tracking wheel
 * @param   updateRateMilliseconds          The desired time between cycles of the odometry loop, in milliseconds, generally 5 or 10
 */
odom::odom(vex::encoder &verticalEncoder, vex::encoder &horizontalEncoder, vex::inertial &Inertial, \
    float verticalDistanceFromCenter, float verticalInchesPerDegree, float horizontalDistanceFromCenter, \
        float horizontalInchesPerDegree, int updateRateMilliseconds){
            
    this->verticalEncoder = &verticalEncoder;
    this->horizontalEncoder = &horizontalEncoder;
    this->Inertial = &Inertial;
    this->verticalDistanceFromCenter = verticalDistanceFromCenter;
    this->verticalInchesPerDegree = verticalInchesPerDegree;
    this->horizontalDistanceFromCenter = horizontalDistanceFromCenter;
    this->horizontalInhcesPerDegree = horizontalInchesPerDegree;
    this->updateRateMilliseconds = updateRateMilliseconds;
    this->usesRotation = false;
}

/**
 * Defualt constructor
 */
odom::odom(){};

/**
 * Getter for the full robot position
 * 
 * @return  std::vector<float> containing robot position: (x, y, heading) in inches and degrees
 */
std::vector<float> odom::getPosition(){
    return this->robotPosition;
}

/**
 * Getter for the robot x position
 * 
 * @return  float containing robot x position in inches
 */
float odom::getX(){
    return this->robotPosition.at(0);
}

/**
 * Getter for the robot y position
 * 
 * @return  float containing robot y position in inches
 */
float odom::getY(){
    return this->robotPosition.at(1);
}

/**
 * Getter for the robot heading
 * 
 * @return  float containing robot heading in degrees
 */
float odom::getHeading(){
    return this->robotPosition.at(2);
}

/**
 * Sets the position of the robot
 * Sets the heading and rotation of the V5 Inertial Sensor
 * 
 * @param   x       the new robot x position in inches
 * @param   y       the new robot y position in inches
 * @param   heading the new robot heading in degrees
 */
void odom::setPosition(float x, float y, float heading){
    this->robotPosition.at(0) = x;
    this->robotPosition.at(1) = y;
    this->robotPosition.at(2) = heading;
    this->Inertial->setHeading(heading, vex::rotationUnits::deg);
    this->Inertial->setRotation(heading, vex::rotationUnits::deg);
}

/**
 * Sets the x position of the robot
 * 
 * @param   x   new robot x position in inches
 */
void odom::setX(float x){
    this->robotPosition.at(0) = x;
}

/**
 * Sets the y position of the robot
 * 
 * @param   y   new robot y position in inches
 */
void odom::setY(float y){
    this->robotPosition.at(1) = y;
}

/**
 * Sets the heading of the robot
 * Sets the heading and rotation of the V5 Inertial Sensor
 * 
 * @param   heading the new heading of the robot in degrees
 */
void odom::setHeading(float heading){
    this->robotPosition.at(2) = heading;
    this->Inertial->setHeading(heading, vex::rotationUnits::deg);
    this->Inertial->setRotation(heading, vex::rotationUnits::deg);
}