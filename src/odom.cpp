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
 * Default constructor for the odometry_pod class
 */
odometry::odometry_pod::odometry_pod() :
m_sensor(NONE),
m_orientation(VERTICAL),
m_distanceFromTrackingCenter(0.0f)
{
}

/**
 * Constructor method for an odometry pod
 *
 * @param sensor the type of sensor being used
 * @param orientation the orientation of the tracking wheel
 * @param encoder a pointer to the encoder on the tracking wheel
 * @param distanceFromTrackingCenter the perpendicular distance from the wheel to the tracking center of the robot
 *
 * @return an odometry_pod object
 */
odometry::odometry_pod::odometry_pod(wheelOrientation orientation, vex::encoder *encoder, float distanceFromTrackingCenter, float inchesPerDegree) :
    m_sensor(OS_ENCODER),
    m_orientation(orientation),
    m_encoder(encoder),
    m_distanceFromTrackingCenter(distanceFromTrackingCenter),
    m_inchesPerDegree(inchesPerDegree)
{
}

/**
 * Constructor method for an odometry pod
 * 
 * @param sensor the type of sensor being used
 * @param orientation the orientation of the tracking wheel
 * @param rotation a pointer to the rotation sensor on the tracking wheel
 * @param distanceFromTrackingCenter the perpendicular distance from the wheel to the tracking center of the robot
 * 
 * @return an odometry_pod object
 */
odometry::odometry_pod::odometry_pod(wheelOrientation orientation, vex::rotation *rotation, float distanceFromTrackingCenter, float inchesPerDegree) :
    m_sensor(V5_ROTATION),
    m_orientation(orientation),
    m_rotation(rotation),
    m_distanceFromTrackingCenter(distanceFromTrackingCenter),
    m_inchesPerDegree(inchesPerDegree)
{
}

/**
 * Constructor method for an odometry pod
 * 
 * @param sensor the type of sensor being used
 * @param orientation the orientation of the tracking wheel
 * @param motor a pointer to the motor on the tracking wheel
 * @param distanceFromTrackingCenter the perpendicular distance from the wheel to the tracking center of the robot
 * 
 * @return an odometry_pod object
 */
odometry::odometry_pod::odometry_pod(wheelOrientation orientation, vex::motor *motor, float distanceFromTrackingCenter, float inchesPerDegree) :
    m_sensor(V5_MOTOR_ENCODER),
    m_orientation(orientation),
    m_motor(motor),
    m_distanceFromTrackingCenter(distanceFromTrackingCenter),
    m_inchesPerDegree(inchesPerDegree)
{
}

/**
 * @return the perpendicular distance from the tracking wheel to the tracking center of the robot
 */
inline float odometry::odometry_pod::getDistanceFromTrackingCenter()
{
    return m_distanceFromTrackingCenter;
}

/**
 * @return the position of the tracking wheel's sensor
 */
float odometry::odometry_pod::getPosition()
{
    switch (m_sensor)
    {
        case V5_MOTOR_ENCODER:
            return m_motor->position(vex::rotationUnits::deg) * m_inchesPerDegree;
        case V5_ROTATION:
            return m_rotation->position(vex::rotationUnits::deg) * m_inchesPerDegree;
        case OS_ENCODER:
            return m_encoder->position(vex::rotationUnits::deg) * m_inchesPerDegree;   
        default:
            return 0.0f;
    }
}

/**
 * Converts degrees to radians
 * 
 * @param degrees the number of degrees
 * 
 * @return the number of radians
 */
inline float odometry::convertToRadians(float degrees)
{
    return degrees * M_PI / 180.0f;
}

/**
 * Calculates the change in position of the vertical wheel, in inches
 *
 * @param   previousVerticalPosition the variable containing the previous position, this is updated
 *
 * @return the change in position, in inches
 */
float odometry::getVerticalChange(float &previousVerticalPosition)
{
    float currentPosition = m_verticalTrackingWheel.getPosition();
    float changeInPosition = currentPosition - previousVerticalPosition;
    previousVerticalPosition = currentPosition;

    return changeInPosition;
}

/**
 * Calculates the change in position of the horizontal wheel, in inches
 * 
 * @param   previousHorizontalPosition the variable containing the previous position, this is updated
 * 
 * @return the change in position, in inches
 */
float odometry::getHorizontalChange(float &previousHorizontalPosition)
{
    float currentPosition = m_horizontalTrackingWheel.getPosition();
    float changeInPosition = currentPosition - previousHorizontalPosition;
    previousHorizontalPosition = currentPosition;

    return changeInPosition;
}

/**
 * Calculates the average rotation of the Inertial sensor
 * 
 * @param previousRotation the variable containing the previous rotation, this is updated
 * 
 * @return the average rotation, in radians
 */
float odometry::getChangeInRotation(float &previousRotation)
{
    float currentRotation = convertToRadians(m_Inertial->rotation(vex::rotationUnits::deg));
    float changeInRotation = currentRotation - previousRotation;
    previousRotation = currentRotation;

    return changeInRotation;
}

/**
 * Runs the main loop of odometry
 * 
 * @param initalX the starting x-coordinate
 * @param initialY the starting y-coordinate
 * @param initialHeading the startin heading
 */
void odometry::startTracking(float initialX, float initialY, float initialHeading)
{
    g_xPosition = initialX;
    g_yPosition = initialY;
    m_Inertial->setHeading(initialHeading, vex::rotationUnits::deg);
    m_Inertial->setRotation(initialHeading, vex::rotationUnits::deg);

    float previousVerticalPosition = m_verticalTrackingWheel.getPosition();
    float previousHorizontalPosition = m_horizontalTrackingWheel.getPosition();
    float previousRotation = convertToRadians(initialHeading);

    m_tracking = true;

    while(m_tracking)
    {
        float changeInVerticalPosition = getVerticalChange(previousVerticalPosition);
        float changeInHorizontalPosition = getHorizontalChange(previousHorizontalPosition);
        float changeInRotation = getChangeInRotation(previousRotation);

        float localX;
        float localY;
        if(changeInRotation == 0)
        {
            localX = changeInHorizontalPosition;
            localY = changeInVerticalPosition;
        }
        else {
            localX = 2 * sinf(convertToRadians(m_Inertial->rotation(vex::rotationUnits::deg))) * ((changeInHorizontalPosition / changeInRotation) + m_horizontalTrackingWheel.getDistanceFromTrackingCenter());
            localY = 2 * sinf(convertToRadians(m_Inertial->rotation(vex::rotationUnits::deg))) * ((changeInVerticalPosition / changeInRotation) + m_verticalTrackingWheel.getDistanceFromTrackingCenter());
        }

        float averageRotation = convertToRadians(m_Inertial->rotation(vex::rotationUnits::deg)) - (changeInRotation / 2);

        float polarRadius = hypotf(localX, localY);
        float globalPolarAngle = atan2f(localX, localY) - averageRotation;

        g_xPosition += cosf(globalPolarAngle) * polarRadius;
        g_yPosition += sinf(globalPolarAngle) * polarRadius;

        vex::task::sleep(10);
    }
}

/**
 * Stops the main odometry loop
 */
void odometry::stopTracking()
{
    m_tracking = false;
}

/**
 * @return the position of the robot {x, y, heading} as {inches, inches, degrees}
 */
std::vector<float> odometry::getPosition()
{
    return {g_xPosition, g_yPosition, (float)m_Inertial->heading(vex::rotationUnits::deg)};
}

/**
 * Constructor method for the odometry class
 * 
 * @param verticalTrackingWheel an odometry_pod object representing the vertical tracking wheel
 * @param horizontalTrackingWheel an odometry_pod object representing the horizontal tracking wheel
 */
odometry::odometry(odometry_pod verticalTrackingWheel, odometry_pod horizontalTrackingWheel, vex::inertial* Inertial) :
m_verticalTrackingWheel(verticalTrackingWheel),
m_horizontalTrackingWheel(horizontalTrackingWheel),
m_Inertial(Inertial)
{
}
