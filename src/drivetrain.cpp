#include "drivetrain.h"

/**
 * Private function that restrains a number to a given range
 * This keeps the number's relative value
 * 
 * @param   num the number to be restrained
 * @param   min the minimum acceptable value
 * @param   max the maximum acceptable value
 * 
 * @return  the restrained number
 */
float chassis::restrain(float num, float min, float max)
{
    while(num > max) num -= (max - min);
    while(num < min) num += (max - min);
    return num;
}

/**
 * Private function that clamps a number toa given range
 * This does not keep the number's relative value
 * 
 * @param   num the number to be clamped
 * @param   min the minimum acceptable value
 * @param   max the maximum acceptable value
 * 
 * @return  the clamped number
 */
float chassis::clamp(float num, float min, float max)
{
    if(num > max) return max;
    if(num < min) return min;
    return num;
}

/**
 * Private function that converts radians to degrees
 */
float chassis::radToDeg(float rad)
{
    return (rad / M_PI) * 180;
}

/**
 * Constructor method using internal motor encoders
 * 
 * @param   getRobotPosition    a function that returns a std::vector<float> representing the robot's position
 * @param   Left                a pointer to the left motor group of the drivetrain
 * @param   Right               a pointer to the right motor group of the drivetrain
 * @param   Inertial            a pointer to the v5 inertial sensor on the drivetrain
 * @param   trackWidth          the trackwidth of the robot's drivetrain
 * @param   degreesToInches     a ratio to convert degrees to inches
 */
chassis::chassis(std::function<std::vector<float>()> getRobotPosition, vex::motor_group *Left, vex::motor_group *Right, vex::inertial *Inertial, float trackWidth, float degreesToInches)
{
    this->getRobotPosition = getRobotPosition;
    this->Left = Left;
    this->Right = Right;
    this->Inertial = Inertial;
    this->trackWidth = trackWidth;
    this->degreesToInches = degreesToInches;
    this->trackingType = verticalTracking::motorEncoder;
}

/**
 * Constructor method using a 3-Wire Optical Shaft Encoder
 * 
 * @param   getRobotPosition    a function that returns a std::vector<float> representing the robot's position
 * @param   Left                a pointer to the left motor group of the drivetrain
 * @param   Right               a pointer to the right motor group of the drivetrain
 * @param   Inertial            a pointer to the v5 inertial sensor on the drivetrain
 * @param   VerticalEncoder     a pointer to the vertical opitcal shaft encoder
 * @param   trackWidth          the trackwidth of the robot's drivetrain
 * @param   degreesToInches     a ratio to convert degrees to inches
 */
chassis::chassis(std::function<std::vector<float>()> getRobotPosition, vex::motor_group *Left, vex::motor_group *Right, vex::inertial* Inertial, vex::encoder *VerticalEncoder, float trackWidth,float degreesToInches)
{
    this->getRobotPosition = getRobotPosition;
    this->Left = Left;
    this->Right = Right;
    this->Inertial = Inertial;
    this->VerticalEncoder = VerticalEncoder;
    this->trackWidth = trackWidth;
    this->degreesToInches = degreesToInches;
    this->trackingType = verticalTracking::encoder;
}

/**
 * Constructor method using a v5 rotation sensor
 * 
 * @param   getRobotPosition    a function that returns a std::vector<float> representing the robot's position
 * @param   Left                a pointer to the left motor group of the drivetrain
 * @param   Right               a pointer to the right motor group of the drivetrain
 * @param   Inertial            a pointer to the v5 inertial sensor on the drivetrain
 * @param   VerticalRotation    a pointer to the vertical v5 rotation sensor
 * @param   trackWidth          the trackwidth of the robot's drivetrain
 * @param   degreesToInches     a ratio to convert degrees to inches
 */
chassis::chassis(std::function<std::vector<float>()> getRobotPosition, vex::motor_group *Left, vex::motor_group *Right, vex::inertial* Inertial, vex::rotation *VerticalRotation, float trackWidth, float degreesToInches)
{
    this->getRobotPosition = getRobotPosition;
    this->Left = Left;
    this->Right = Right;
    this->Inertial = Inertial;
    this->VerticalRotation = VerticalRotation;
    this->trackWidth = trackWidth;
    this->degreesToInches = degreesToInches;
    this->trackingType = verticalTracking::rotation;
}

/**
 * Tunes the constants of the drive PID
 * 
 * @param   Kd                  proportional constant
 * @param   Ki                  integral constant
 * @param   Kd                  derivative constant
 * @param   integralTolerance   integral tolerance
 * @param   settleTolerance     settle tolerance
 * @param   settleTime          required settle time
 * @param   minOutput           minimum acceptable output
 * @param   maxOutput           maximum acceptable output
 * @param   headingKp           proprotional constant to hold heading
 */
void chassis::setDriveConstants(float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput, float headingKp)
{
    this->driveConstants.Kp = Kp;
    this->driveConstants.Ki = Ki;
    this->driveConstants.Kd = Kd;
    this->driveConstants.integralTolerance = integralTolerance;
    this->driveConstants.settleTolerance = settleTolerance;
    this->driveConstants.settleTime = settleTime;
    this->driveConstants.minOutput = minOutput;
    this->driveConstants.maxOutput = maxOutput;
    this->driveConstants.headingKp = headingKp;
}

/**
 * Tunes the constants of the turn PID
 * 
 * @param   Kd                  proportional constant
 * @param   Ki                  integral constant
 * @param   Kd                  derivative constant
 * @param   integralTolerance   integral tolerance
 * @param   settleTolerance     settle tolerance
 * @param   settleTime          required settle time
 * @param   minOutput           minimum acceptable output
 * @param   maxOutput           maximum acceptable output
 */
void chassis::setTurnConstants(float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput)
{
    this->turnConstants.Kp = Kp;
    this->turnConstants.Ki = Ki;
    this->turnConstants.Kd = Kd;
    this->turnConstants.integralTolerance = integralTolerance;
    this->turnConstants.settleTolerance = settleTolerance;
    this->turnConstants.settleTime = settleTime;
    this->turnConstants.minOutput = minOutput;
    this->turnConstants.maxOutput = maxOutput;
}

/**
 * Tunes the constants of the swing PID
 * 
 * @param   Kd                  proportional constant
 * @param   Ki                  integral constant
 * @param   Kd                  derivative constant
 * @param   integralTolerance   integral tolerance
 * @param   settleTolerance     settle tolerance
 * @param   settleTime          required settle time
 * @param   minOutput           minimum acceptable output
 * @param   maxOutput           maximum acceptable output
 */
void chassis::setSwingConstants(float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput)
{
    this->swingConstants.Kp = Kp;
    this->swingConstants.Ki = Ki;
    this->swingConstants.Kd = Kd;
    this->swingConstants.integralTolerance = integralTolerance;
    this->swingConstants.settleTolerance = settleTolerance;
    this->swingConstants.settleTime = settleTime;
    this->swingConstants.minOutput = minOutput;
    this->swingConstants.maxOutput = maxOutput;
}

/**
 * Tunes the constants of the arc PID
 * 
 * @param   Kd                  proportional constant
 * @param   Ki                  integral constant
 * @param   Kd                  derivative constant
 * @param   integralTolerance   integral tolerance
 * @param   settleTolerance     settle tolerance
 * @param   settleTime          required settle time
 * @param   minOutput           minimum acceptable output
 * @param   maxOutput           maximum acceptable output
 */
void chassis::setArcConstants(float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput)
{
    this->arcConstants.Kp = Kp;
    this->arcConstants.Ki = Ki;
    this->arcConstants.Kd = Kd;
    this->arcConstants.integralTolerance = integralTolerance;
    this->arcConstants.settleTolerance = settleTolerance;
    this->arcConstants.settleTime = settleTime;
    this->arcConstants.minOutput = minOutput;
    this->arcConstants.maxOutput = maxOutput;
}

/**
 * Drives for a distance using a PID with no timeout
 * 
 * @param   distance    the distance to be driven, in inches
 * 
 * @return  the time it takes for the PID to settle
 */
float chassis::driveFor(float distance)
{
    return this->driveFor(distance, INFINITY, 0, this->driveConstants.Kp, this->driveConstants.Ki, this->driveConstants.Kd, this->driveConstants.integralTolerance, this->driveConstants.settleTolerance, this->driveConstants.settleTime, this->driveConstants.minOutput, this->driveConstants.maxOutput, 0);
}

/**
 * Drives for a distance using a PID with a timeout
 * 
 * @param   distance    the distance to be driven, in inches
 * @param   timeout     the time before the drive gives up, in seconds
 * 
 * @return  the time it takes for the PID to settle or time out
 */
float chassis::driveFor(float distance, float timeout)
{
    return this->driveFor(distance, timeout, 0, this->driveConstants.Kp, this->driveConstants.Ki, this->driveConstants.Kd, this->driveConstants.integralTolerance, this->driveConstants.settleTolerance, this->driveConstants.settleTime, this->driveConstants.minOutput, this->driveConstants.maxOutput, 0);
}

/**
 * Drives for a distance using a PID with a timeout
 * This will hold a heading as it drives
 * 
 * @param   distance    the distance to be drive, in inches
 * @param   timeout     the time before the drive gives up, in seconds
 * @param   heading     the desired heading for the robot to hold
 * 
 * @return  the time it takes for the PID to settle or time out
 */
float chassis::driveFor(float distance, float timeout, float heading)
{
    return this->driveFor(distance, timeout, heading, this->driveConstants.Kp, this->driveConstants.Ki, this->driveConstants.Kd, this->driveConstants.integralTolerance, this->driveConstants.settleTolerance, this->driveConstants.settleTime, this->driveConstants.minOutput, this->driveConstants.maxOutput, this->driveConstants.headingKp);
}

/**
 * Drives for a distance using a PID with a timeout
 * 
 * @param   distance            the distance to be driven, in inches
 * @param   timeout             the time before the drive gives up, in seconds
 * @param   Kp                  the proportional constant
 * @param   Ki                  the integral constant
 * @param   Kd                  the derivative constant
 * @param   integralTolerance   the tolerance range for the integral to grow, in inches
 * @param   settleTolerance     the tolerance range for the PID to be considered settled, in inches
 * @param   settleTime          the amount of time the error must be within the settleTolerance before it is truly settled, in seconds
 * @param   minOutput           the minimum acceptable output, in volts
 * @param   maxOutput           the maximum acceptable output, in volts
 */
float chassis::driveFor(float distance, float timeout, float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput)
{
    return this->driveFor(distance, timeout, 0, Kp, Ki, Kd, integralTolerance, settleTolerance, settleTime, minOutput, maxOutput, 0);
}

/**
 * Drives for a distance using a PID with a timeout
 * This will hold a heading as it drives
 * 
 * @param   distance            the distance to be driven, in inches
 * @param   timeout             the time before the drive gives up, in seconds
 * @param   heading             the desired heading for the robot to hold
 * @param   Kp                  the proportional constant
 * @param   Ki                  the integral constant
 * @param   Kd                  the derivative constant
 * @param   integralTolerance   the tolerance range for the integral to grow, in inches
 * @param   settleTolerance     the tolerance range for the PID to be considered settled, in inches
 * @param   settleTime          the amount of time the error must be within the settleTolerance before it is truly settled, in seconds
 * @param   minOutput           the minimum acceptable output, in volts
 * @param   maxOutput           the maximum acceptable output, in volts
 * @param   headingKp           the proportional constant for the heading PID
 */
float chassis::driveFor(float distance, float timeout, float heading, float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput, float headingKp)
{
    PID drivePID = PID(Kp, Ki, Kd, integralTolerance, settleTolerance, settleTime, minOutput, maxOutput, 10);
    PID turnPID = PID(headingKp, 0, 0, 0, 0, 0, minOutput, maxOutput, 10);

    float initialPosition;
    if(this->trackingType == chassis::verticalTracking::rotation) initialPosition = this->VerticalRotation->position(vex::rotationUnits::deg) * this->degreesToInches;
    else if(this->trackingType == chassis::verticalTracking::encoder) initialPosition = this->VerticalEncoder->position(vex::rotationUnits::deg) * this->degreesToInches;
    else initialPosition = this->Left->position(vex::rotationUnits::deg) * this->degreesToInches;

    float t = 0;
    while(!drivePID.isSettled() && t < timeout){
        float currentPosition;
        if(this->trackingType == chassis::verticalTracking::rotation) currentPosition = this->VerticalRotation->position(vex::rotationUnits::deg) * this->degreesToInches;
        else if(this->trackingType == chassis::verticalTracking::encoder) currentPosition = this->VerticalEncoder->position(vex::rotationUnits::deg) * this->degreesToInches;
        else currentPosition = this->Left->position(vex::rotationUnits::deg) * this->degreesToInches;

        float driveError = distance - (currentPosition - initialPosition);
        float headingError = this->restrain(this->Inertial->heading(vex::rotationUnits::deg) - heading, -180, 180);

        float driveOutput = drivePID.getOutput(driveError);
        float turnOutput = turnPID.getOutput(headingError);

        this->Left->spin(vex::directionType::fwd, this->clamp(driveOutput + turnOutput, minOutput, maxOutput), vex::voltageUnits::volt);
        this->Right->spin(vex::directionType::fwd, this->clamp(driveOutput - turnOutput, minOutput, maxOutput), vex::voltageUnits::volt);

        vex::task::sleep(10);
        t += 0.01;
    }

    this->stopDrive(vex::brakeType::hold);

    return t;
}

/**
 * Sets the speed of the drivetrain
 * 
 * @param   speed   the desired speed
 * @param   unit    the unit of the speed
 */
void chassis::setDriveSpeed(float speed, vex::velocityUnits unit)
{
    this->Left->spin(vex::directionType::fwd, speed, unit);
    this->Right->spin(vex::directionType::fwd, speed, unit);
}

/**
 * Sets the speed of the drivetrain
 * 
 * @param   speed   the desired speed
 * @param   unit    the unit of the speed
 */
void chassis::setDriveSpeed(float speed, vex::voltageUnits unit)
{
    this->Left->spin(vex::directionType::fwd, speed, unit);
    this->Right->spin(vex::directionType::fwd, speed, unit);
}

/**
 * Stops the drivetrain using the desired stopping type
 * 
 * @param   stopType    the desired stop type
 */
void chassis::stopDrive(vex::brakeType stopType)
{
    this->Left->stop(stopType);
    this->Right->stop(stopType);
}

/**
 * Drives to a position on the field using a turn PID and a drive PID, without timeouts
 * Requires odometry to be active
 * 
 * @param   x   the desired x coordinate, in inches
 * @param   y   the desired y coordinate, in inches
 * 
 * @return  the total time it takes to reach (x, y)
 */
float chassis::driveTo(float x, float y)
{
    float timeTurning = turnToPosition(x, y);

    std::vector<float> robotPosition;
    float distance = hypotf(robotPosition.at(0) - x, robotPosition.at(1) - y);
    float timeDriving = driveFor(distance, INFINITY, this->Inertial->heading(vex::rotationUnits::deg));

    return timeTurning + timeDriving;
}

/**
 * Drives to a position on the field using a turn PID and a drive PID, with timeouts
 * Requires odometry to be active
 * 
 * @param   x               the desired x coordinate, in inches
 * @param   y               the desired y coordinate, in inches
 * @param   driveTimeout    the amount of time before the drive PID gives up, in seconds
 * @param   turnTimeout     the amount of time before the turn PID gives up, in seconds
 * 
 * @return  the total time it takes to reach (x, y)
 */
float chassis::driveTo(float x, float y, float driveTimeout, float turnTimeout)
{
    float timeTurning = turnToPosition(x, y, turnTimeout);

    std::vector<float> robotPosition;
    float distance = hypotf(robotPosition.at(0) - x, robotPosition.at(1) - y);
    float timeDriving = driveFor(distance, driveTimeout, this->Inertial->heading(vex::rotationUnits::deg));

    return timeTurning + timeDriving;
}

/**
 * Drives backwards to a position on the field using a turn PID and a drive PID, without timeouts
 * Requires odometry to be active
 * 
 * @param   x   the desired x coordinate, in inches
 * @param   y   the desired y coordinate, in inches
 * 
 * @return  the total time it takes to reach (x, y)
 */
float chassis::driveToReverse(float x, float y)
{
    float timeTurning = turnToPositionReverse(x, y);

    std::vector<float> robotPosition;
    float distance = -hypotf(robotPosition.at(0) - x, robotPosition.at(1) - y);
    float timeDriving = driveFor(distance, INFINITY, this->Inertial->heading(vex::rotationUnits::deg));

    return timeTurning + timeDriving;
}

/**
 * Drives backwards to a position on the field using a turn PID and a drive PID, with timeouts
 * Requires odometry to be active
 * 
 * @param   x               the desired x coordinate, in inches
 * @param   y               the desired y coordinate, in inches
 * @param   driveTimeout    the amount of time before the drive PID gives up, in seconds
 * @param   turnTimeout     the amount of time before the turn PID gives up, in seconds
 * 
 * @return  the total time it takes to reach (x, y)
 */
float chassis::driveToReverse(float x, float y, float driveTimeout, float turnTimeout)
{
    float timeTurning = turnToPositionReverse(x, y, turnTimeout);

    std::vector<float> robotPosition;
    float distance = -hypotf(robotPosition.at(0) - x, robotPosition.at(1) - y);
    float timeDriving = driveFor(distance, driveTimeout, this->Inertial->heading(vex::rotationUnits::deg));

    return timeTurning + timeDriving;
}

/**
 * Turns for a specified number of degrees using a PID without a timeout
 * 
 * @param   degrees the number of degres to be turned
 * 
 * @return  the time it takes for the PID to settle
 */
float chassis::turnFor(float degrees)
{
    return this->turnFor(degrees, INFINITY, this->turnConstants.Kp, this->turnConstants.Ki, this->turnConstants.Kd, this->turnConstants.integralTolerance, this->turnConstants.settleTolerance, this->turnConstants.settleTime, this->turnConstants.minOutput, this->turnConstants.maxOutput);
}

/**
 * Turns for a specified number of degrees using a PID with a timeout
 * 
 * @param   degrees the number of degrees to be turned
 * @param   timeout the time before the PID gives up
 * 
 * @return  the time it takes for the PID to settle or time out
 */
float chassis::turnFor(float degrees, float timeout)
{
    return this->turnFor(degrees, timeout, this->turnConstants.Kp, this->turnConstants.Ki, this->turnConstants.Kd, this->turnConstants.integralTolerance, this->turnConstants.settleTolerance, this->turnConstants.settleTime, this->turnConstants.minOutput, this->turnConstants.maxOutput);
}

/**
 * Turns for a specified number of degrees using a PID with a timeout
 * 
 * @param   degrees             the number of degrees to be turned
 * @param   timeout             the time before the PID gives up, in seconds
 * @param   Kp                  the proportional constant
 * @param   Ki                  the integral constant
 * @param   Kd                  the derivative constant
 * @param   integralTolerance   the tolerance range for the integral to grow, in degrees
 * @param   settleTolerance     the tolerance range for the PID to be considered settled, in degrees
 * @param   settleTime          the amount of time the error must be within the settleTolerance before it is truly settled, in seconds
 * @param   minOutput           the minimum acceptable output, in volts
 * @param   maxOutput           the maximum acceptable output, in volts
 * 
 * @return  the time it takes for the PID to settle or time out
 */
float chassis::turnFor(float degrees, float timeout, float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput)
{
    PID turnPID = PID(Kp, Ki, Kd, integralTolerance, settleTolerance, settleTime, minOutput, maxOutput, 10);
    float targetRotation = this->Inertial->rotation(vex::rotationUnits::deg) + degrees;

    float t = 0;
    while(!turnPID.isSettled() && t < timeout){
        float error = targetRotation - this->Inertial->rotation(vex::rotationUnits::deg);
        float output = turnPID.getOutput(error);

        this->Left->spin(vex::directionType::fwd, output, vex::voltageUnits::volt);
        this->Right->spin(vex::directionType::rev, output, vex::voltageUnits::volt);

        vex::task::sleep(10);
        t += 0.01;
    }

    this->stopDrive(vex::brakeType::hold);

    return t;
}

/**
 * Turns to a specified heading using a PID and no timeout
 * 
 * @param   heading desired heading, in degrees
 * 
 * @return  the time it takes for the PID to settle
 */
float chassis::turnTo(float heading)
{
    return turnTo(heading, INFINITY, this->turnConstants.Kp, this->turnConstants.Ki, this->turnConstants.Kd, this->turnConstants.integralTolerance, this->turnConstants.settleTolerance, this->turnConstants.settleTime, this->turnConstants.minOutput, this->turnConstants.maxOutput);
}

/**
 * Turns to a specified heading using a PID and a timeout
 * 
 * @param   heading desired heading, in degrees
 * @param   timeout time before the PID gives up, in seconds
 * 
 * @return  the time it takes for the PID to settle or time out
 */
float chassis::turnTo(float heading, float timeout)
{
    return turnTo(heading, timeout, this->turnConstants.Kp, this->turnConstants.Ki, this->turnConstants.Kd, this->turnConstants.integralTolerance, this->turnConstants.settleTolerance, this->turnConstants.settleTime, this->turnConstants.minOutput, this->turnConstants.maxOutput);
}

/**
 * Turns to a specified heading using a PID and a timeout
 * 
 * @param   heading             the desired heading, in degrees
 * @param   timeout             the time before the PID gives up, in seconds
 * @param   Kp                  the proportional constant
 * @param   Ki                  the integral constant
 * @param   Kd                  the derivative constant
 * @param   integralTolerance   the tolerance range for the integral to grow, in degrees
 * @param   settleTolerance     the tolerance range for the PID to be considered settled, in degrees
 * @param   settleTime          the amount of time the error must be within the settleTolerance before it is truly settled, in seconds
 * @param   minOutput           the minimum acceptable output, in volts
 * @param   maxOutput           the maximum acceptable output, in volts
 * 
 * @return  the time it takes for the PID to settle or time out
 */
float chassis::turnTo(float heading, float timeout, float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput)
{
    PID turnPID = PID(Kp, Ki, Kd, integralTolerance, settleTolerance, settleTime, minOutput, maxOutput, 10);

    float t = 0;
    while(!turnPID.isSettled() && t < timeout){
        float error = this->restrain(heading - this->Inertial->heading(vex::rotationUnits::deg), -180, 180);
        float output = turnPID.getOutput(error);

        this->Left->spin(vex::directionType::fwd, output, vex::voltageUnits::volt);
        this->Right->spin(vex::directionType::rev, output, vex::voltageUnits::volt);

        vex::task::sleep(10);
        t += 0.01;
    }

    this->stopDrive(vex::brakeType::hold);

    return t;
}

/**
 * Turns to a position on the field using a PID without a timeout
 * 
 * @param   x x coordinate, in inches
 * @param   y y coordinate, in incehs
 * 
 * @return  the time it takes the PID to settle
 */
float chassis::turnToPosition(float x, float y)
{
    std::vector<float> robotPosition = this->getRobotPosition();
    float targetHeading = this->radToDeg(atan2f(robotPosition.at(0) - x, robotPosition.at(1) - y));

    return turnTo(targetHeading, INFINITY, this->turnConstants.Kp, this->turnConstants.Ki, this->turnConstants.Kd, this->turnConstants.integralTolerance, this->turnConstants.settleTolerance, this->turnConstants.settleTime, this->turnConstants.minOutput, this->turnConstants.maxOutput);
}

/**
 * Turns to a position on the field using a PID with a timeout
 * 
 * @param   x       x coordinate, in inches
 * @param   y       y coordinate, in incehs
 * @param   timeout the amount of time before the turn gives up
 * 
 * @return  the time it takes the PID to settle
 */
float chassis::turnToPosition(float x, float y, float timeout)
{
    std::vector<float> robotPosition = this->getRobotPosition();
    float targetHeading = this->radToDeg(atan2f(robotPosition.at(0) - x, robotPosition.at(1) - y));

    return turnTo(targetHeading, timeout, this->turnConstants.Kp, this->turnConstants.Ki, this->turnConstants.Kd, this->turnConstants.integralTolerance, this->turnConstants.settleTolerance, this->turnConstants.settleTime, this->turnConstants.minOutput, this->turnConstants.maxOutput);
}

/**
 * Turns to a specified psoition using a PID and a timeout
 * 
 * @param   x                   the x coordinate, in iches
 * @param   y                   the y coordinate, in inches
 * @param   timeout             the time before the PID gives up, in seconds
 * @param   Kp                  the proportional constant
 * @param   Ki                  the integral constant
 * @param   Kd                  the derivative constant
 * @param   integralTolerance   the tolerance range for the integral to grow, in degrees
 * @param   settleTolerance     the tolerance range for the PID to be considered settled, in degrees
 * @param   settleTime          the amount of time the error must be within the settleTolerance before it is truly settled, in seconds
 * @param   minOutput           the minimum acceptable output, in volts
 * @param   maxOutput           the maximum acceptable output, in volts
 * 
 * @return  the time it takes for the PID to settle or time out
 */
float chassis::turnToPosition(float x, float y, float timeout, float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput)
{
    std::vector<float> robotPosition = this->getRobotPosition();
    float targetHeading = this->radToDeg(atan2f(robotPosition.at(0) - x, robotPosition.at(1) - y));

    return turnTo(targetHeading, timeout, Kp, Ki, Kd, integralTolerance, settleTolerance, settleTime, minOutput, maxOutput);
}

/**
 * Turns to a position on the field using a PID without a timeout
 * 
 * @param   x x coordinate, in inches
 * @param   y y coordinate, in incehs
 * 
 * @return  the time it takes the PID to settle
 */
float chassis::turnToPositionReverse(float x, float y)
{
    std::vector<float> robotPosition = this->getRobotPosition();
    float targetHeading = this->radToDeg(atan2f(robotPosition.at(0) - x, robotPosition.at(1) - y)) + 180;

    return turnTo(targetHeading, INFINITY, this->turnConstants.Kp, this->turnConstants.Ki, this->turnConstants.Kd, this->turnConstants.integralTolerance, this->turnConstants.settleTolerance, this->turnConstants.settleTime, this->turnConstants.minOutput, this->turnConstants.maxOutput);
}

/**
 * Turns to a position on the field using a PID with a timeout
 * 
 * @param   x       x coordinate, in inches
 * @param   y       y coordinate, in incehs
 * @param   timeout the amount of time before the turn gives up
 * 
 * @return  the time it takes the PID to settle
 */
float chassis::turnToPositionReverse(float x, float y, float timeout)
{
    std::vector<float> robotPosition = this->getRobotPosition();
    float targetHeading = this->radToDeg(atan2f(robotPosition.at(0) - x, robotPosition.at(1) - y)) + 180;

    return turnTo(targetHeading, timeout, this->turnConstants.Kp, this->turnConstants.Ki, this->turnConstants.Kd, this->turnConstants.integralTolerance, this->turnConstants.settleTolerance, this->turnConstants.settleTime, this->turnConstants.minOutput, this->turnConstants.maxOutput);
}

/**
 * Sets the turn velocity of the drivetrain
 * 
 * @param   speed   the desired speed
 * @param   unit    speed's unit
 */
void chassis::setTurnSpeed(float speed, vex::velocityUnits unit)
{
    this->Left->spin(vex::directionType::fwd, speed, unit);
    this->Right->spin(vex::directionType::rev, speed, unit);
}

/**
 * Sets the turn velocity of the drivetrain
 * 
 * @param   speed   the desired speed
 * @param   unit    speed's unit
 */
void chassis::setTurnSpeed(float speed, vex::voltageUnits unit)
{
    this->Left->spin(vex::directionType::fwd, speed, unit);
    this->Right->spin(vex::directionType::rev, speed, unit);
}

/**
 * Swings for a specified number of degrees in the specified direction using a PID without a timeout
 * A swing sets one side of the drivetrain to hold and the other receieves power
 * 
 * @param   direction   the desired direction
 * @param   degrees     the desired number of degrees
 * 
 * @return  the time it takes the PID to settle
 */
float chassis::swingFor(vex::turnType direction, float degrees)
{
    return this->swingFor(direction, degrees, INFINITY, this->swingConstants.Kp, this->swingConstants.Ki, this->swingConstants.Kd, this->swingConstants.integralTolerance, this->swingConstants.settleTolerance, this->swingConstants.settleTime, this->swingConstants.minOutput, this->swingConstants.maxOutput);
}

/**
 * Swings for a specified number of degrees in the specified direction using a PID with a timeout
 * A swing sets one side of the drivetrain to hold and the other receieves power
 * 
 * @param   direction   the desired direction
 * @param   degrees     the desired number of degrees
 * @param   timeout     the amount of time before the swing gives up, in seconds
 * 
 * @return  the time it takes the PID to settle or time out
 */
float chassis::swingFor(vex::turnType direction, float degrees, float timeout)
{
    return this->swingFor(direction, degrees, timeout, this->swingConstants.Kp, this->swingConstants.Ki, this->swingConstants.Kd, this->swingConstants.integralTolerance, this->swingConstants.settleTolerance, this->swingConstants.settleTime, this->swingConstants.minOutput, this->swingConstants.maxOutput);
}

/**
 * Swings for a specified number of degrees in a specified direction
 * A swing sets one side of the drivetrain to hold and the other receieves power
 * 
 * @param   direction           the direction of the swing
 * @param   degrees             the desired number of degrees
 * @param   timeout             the time before the PID gives up, in seconds
 * @param   Kp                  the proportional constant
 * @param   Ki                  the integral constant
 * @param   Kd                  the derivative constant
 * @param   integralTolerance   the tolerance range for the integral to grow, in degrees
 * @param   settleTolerance     the tolerance range for the PID to be considered settled, in degrees
 * @param   settleTime          the amount of time the error must be within the settleTolerance before it is truly settled, in seconds
 * @param   minOutput           the minimum acceptable output, in volts
 * @param   maxOutput           the maximum acceptable output, in volts
 * 
 * @return  the time it takes for the PID to settle or time out
 */
float chassis::swingFor(vex::turnType direction, float degrees, float timeout, float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput)
{
    PID swingPID = PID(Kp, Ki, Kd, integralTolerance, settleTolerance, settleTime, minOutput, maxOutput, 10);

    float t = 0;
    if(direction == vex::turnType::right){
        float targetRotation = this->Inertial->rotation(vex::rotationUnits::deg) + degrees;
        
        this->Right->stop(vex::brakeType::hold);
        while(!swingPID.isSettled() && t < timeout){
            float error = targetRotation - this->Inertial->rotation(vex::rotationUnits::deg);
            float output = swingPID.getOutput(error);

            this->Left->spin(vex::directionType::fwd, output, vex::voltageUnits::volt);

            vex::task::sleep(10);
            t += 0.01;
        }
    }
    else{
        float targetRotation = this->Inertial->rotation(vex::rotationUnits::deg) - degrees;

        this->Left->stop(vex::brakeType::hold);
        while(!swingPID.isSettled() && t < timeout){
            float error = this->Inertial->rotation(vex::rotationUnits::deg) - targetRotation;
            float output = swingPID.getOutput(error);

            this->Right->spin(vex::directionType::fwd, output, vex::voltageUnits::volt);

            vex::task::sleep(10);
            t += 0.01;
        }
    }

    this->stopDrive(vex::brakeType::hold);

    return t;
}

/**
 * Swings to a specified heading in a specified direction using a PID without a timeout
 * A swing sets one side of the drivetrain to hold and the other receieves power
 * 
 * @param   direction   the desired direction
 * @param   heading     the desired heading, in degrees
 * 
 * @return  the time it takes the PID to settle
 */
float chassis::swingTo(vex::turnType direction, float heading)
{
    return this->swingTo(direction, heading, INFINITY, this->swingConstants.Kp, this->swingConstants.Ki, this->swingConstants.Kd, this->swingConstants.integralTolerance, this->swingConstants.settleTolerance, this->swingConstants.settleTime, this->swingConstants.minOutput, this->swingConstants.maxOutput);
}

/**
 * Swings to a specified heading in a specified direction using a PID with a timeout
 * A swing sets one side of the drivetrain to hold and the other receieves power
 * 
 * @param   direction   the desired direction
 * @param   heading     the desired heading, in degrees
 * @param   timeout     the amount of time before the swing gives up, in seconds
 * 
 * @return  the time it takes the PID to settle or give up
 */
float chassis::swingTo(vex::turnType direction, float heading, float timeout)
{
    return this->swingTo(direction, heading, timeout, this->swingConstants.Kp, this->swingConstants.Ki, this->swingConstants.Kd, this->swingConstants.integralTolerance, this->swingConstants.settleTolerance, this->swingConstants.settleTime, this->swingConstants.minOutput, this->swingConstants.maxOutput);
}

/**
 * Swings To a specified heading in a specified direction
 * A swing sets one side of the drivetrain to hold and the other receieves power
 * 
 * @param   direction           the direction of the swing
 * @param   heading             the desired heading, in degrees
 * @param   timeout             the time before the PID gives up, in seconds
 * @param   Kp                  the proportional constant
 * @param   Ki                  the integral constant
 * @param   Kd                  the derivative constant
 * @param   integralTolerance   the tolerance range for the integral to grow, in degrees
 * @param   settleTolerance     the tolerance range for the PID to be considered settled, in degrees
 * @param   settleTime          the amount of time the error must be within the settleTolerance before it is truly settled, in seconds
 * @param   minOutput           the minimum acceptable output, in volts
 * @param   maxOutput           the maximum acceptable output, in volts
 * 
 * @return  the time it takes for the PID to settle or time out
 */
float chassis::swingTo(vex::turnType direction, float heading, float timeout, float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput)
{
    PID swingPID = PID(Kp, Ki, Kd, integralTolerance, settleTolerance, settleTime, minOutput, maxOutput, 10);

    float t = 0;
    if(direction == vex::turnType::right){        
        this->Right->stop(vex::brakeType::hold);
        while(!swingPID.isSettled() && t < timeout){
            float error = this->restrain(heading - this->Inertial->heading(vex::rotationUnits::deg), -180, 180);
            float output = swingPID.getOutput(error);

            this->Left->spin(vex::directionType::fwd, output, vex::voltageUnits::volt);

            vex::task::sleep(10);
            t += 0.01;
        }
    }
    else{
        this->Left->stop(vex::brakeType::hold);
        while(!swingPID.isSettled() && t < timeout){
            float error = this->restrain(this->Inertial->heading(vex::rotationUnits::deg) - heading, -180, 180);
            float output = swingPID.getOutput(error);

            this->Right->spin(vex::directionType::fwd, output, vex::voltageUnits::volt);

            vex::task::sleep(10);
            t += 0.01;
        }
    }

    this->stopDrive(vex::brakeType::hold);

    return t;
}

/**
 * Sets the swing speed of the robot
 * 
 * @param   direction   the desired direction
 * @param   speed       the desired speed
 * @param   unit        speed's unit
 */
void chassis::setSwingSpeed(vex::turnType direction, float speed, vex::velocityUnits unit)
{
    if(direction == vex::turnType::right){
        this->Right->stop(vex::brakeType::hold);
        this->Left->spin(vex::directionType::fwd, speed, unit);
    }
    else{
        this->Left->stop(vex::brakeType::hold);
        this->Right->spin(vex::directionType::fwd, speed, unit);
    }
}

/**
 * Sets the swing speed of the robot
 * 
 * @param   direction   the desired direction
 * @param   speed       the desired speed
 * @param   unit        speed's unit
 */
void chassis::setSwingSpeed(vex::turnType direction, float speed, vex::voltageUnits unit)
{
    if(direction == vex::turnType::right){
        this->Right->stop(vex::brakeType::hold);
        this->Left->spin(vex::directionType::fwd, speed, unit);
    }
    else{
        this->Left->stop(vex::brakeType::hold);
        this->Right->spin(vex::directionType::fwd, speed, unit);
    }
}

/**
 * Arcs for a specified number of degrees in a specified direction at a specified radius using a PID without a timeout
 * 
 * @param   direction   the desired direction
 * @param   radius      the desired radius, in inches
 * @param   degrees     the desired number of degrees
 * 
 * @return  the amount of time it takes the PID to settle
 */
float chassis::arcFor(vex::turnType direction, float radius, float degrees)
{
    return this->arcFor(direction, radius, degrees, INFINITY, this->arcConstants.Kp, this->arcConstants.Ki, this->arcConstants.Kd, this->arcConstants.integralTolerance, this->arcConstants.settleTolerance, this->arcConstants.settleTime, this->arcConstants.minOutput, this->arcConstants.maxOutput);
}

/**
 * Arcs for a specified number of degrees in a specified direction at a specified radius using a PID with a timeout
 * 
 * @param   direction   the desired direction
 * @param   radius      the desired radius, in inches
 * @param   degrees     the desired number of degrees
 * @param   timeout     the amount fo time before the arc gives up, in seconds
 * 
 * @return  the amount of time it takes the PID to settle or time out
 */
float chassis::arcFor(vex::turnType direction, float radius, float degrees, float timeout)
{
    return this->arcFor(direction, radius, degrees, timeout, this->arcConstants.Kp, this->arcConstants.Ki, this->arcConstants.Kd, this->arcConstants.integralTolerance, this->arcConstants.settleTolerance, this->arcConstants.settleTime, this->arcConstants.minOutput, this->arcConstants.maxOutput);
}

/**
 * Arcs for a specified amount of degrees in a specified direction using a PID with a timeout
 * 
 * @param   direction           the direction of the arc
 * @param   radius              the radius of the arc
 * @param   degrees             the number of degrees
 * @param   timeout             the time before the PID gives up, in seconds
 * @param   Kp                  the proportional constant
 * @param   Ki                  the integral constant
 * @param   Kd                  the derivative constant
 * @param   integralTolerance   the tolerance range for the integral to grow, in degrees
 * @param   settleTolerance     the tolerance range for the PID to be considered settled, in degrees
 * @param   settleTime          the amount of time the error must be within the settleTolerance before it is truly settled, in seconds
 * @param   minOutput           the minimum acceptable output, in volts
 * @param   maxOutput           the maximum acceptable output, in volts
 * 
 * @return  the time it takes for the PID to settle or time out
 */
float chassis::arcFor(vex::turnType direction, float radius, float degrees, float timeout, float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput)
{
    PID arcPID = PID(Kp, Ki, Kd, integralTolerance, settleTolerance, settleTime, minOutput, maxOutput, 10);
    float multiplier = (radius - trackWidth/2) / (radius + trackWidth/2);

    float t = 0;
    if(direction == vex::turnType::right){
        float targetRotation = this->Inertial->rotation(vex::rotationUnits::deg) + degrees;

        while(!arcPID.isSettled() && t < timeout){
            float error = targetRotation - this->Inertial->rotation(vex::rotationUnits::deg);
            float output = arcPID.getOutput(error);

            this->Left->spin(vex::directionType::fwd, output, vex::voltageUnits::volt);
            this->Right->spin(vex::directionType::fwd, multiplier * output, vex::voltageUnits::volt);

            vex::task::sleep(10);
            t += 0.01;
        }
    }
    else{
        float targetRotation = this->Inertial->rotation(vex::rotationUnits::deg) - degrees;

        while(!arcPID.isSettled() && t < timeout){
            float error = this->Inertial->rotation(vex::rotationUnits::deg) - targetRotation;
            float output = arcPID.getOutput(error);

            this->Left->spin(vex::directionType::fwd, multiplier * output, vex::voltageUnits::volt);
            this->Right->spin(vex::directionType::fwd, output, vex::voltageUnits::volt);

            vex::task::sleep(10);
            t += 0.01;
        }
    }

    this->stopDrive(vex::brakeType::hold);

    return t;
}

/**
 * Arcs to a specified heading in a specified direction at a specific radius using a PID without a timeout
 * 
 * @param   direction   the desired direction
 * @param   radius      the specified radius
 * @param   heading     the target heading
 * 
 * @return the time it takes for the PID to settle
 */
float chassis::arcTo(vex::turnType direction, float radius, float heading)
{
    return this->arcTo(direction, radius, heading, INFINITY, this->arcConstants.Kp, this->arcConstants.Ki, this->arcConstants.Kd, this->arcConstants.integralTolerance, this->arcConstants.settleTolerance, this->arcConstants.settleTime, this->arcConstants.minOutput, this->arcConstants.maxOutput);
}

/**
 * Arcs to a specified heading in a specified direction at a specific radius using a PID with a timeout
 * 
 * @param   direction   the desired direction
 * @param   radius      the specified radius
 * @param   heading     the target heading
 * @param   timeout     the amount of time before the arc would give up
 * 
 * @return the time it takes for the PID to settle or time out
 */
float chassis::arcTo(vex::turnType direction, float radius, float heading, float timeout)
{
    return this->arcTo(direction, radius, heading, timeout, this->arcConstants.Kp, this->arcConstants.Ki, this->arcConstants.Kd, this->arcConstants.integralTolerance, this->arcConstants.settleTolerance, this->arcConstants.settleTime, this->arcConstants.minOutput, this->arcConstants.maxOutput);
}

/**
 * Arcs for a specified amount of degrees in a specified direction using a PID with a timeout
 * 
 * @param   direction           the direction of the arc
 * @param   radius              the radius of the arc
 * @param   heading             the desired heading, in degrees
 * @param   timeout             the time before the PID gives up, in seconds
 * @param   Kp                  the proportional constant
 * @param   Ki                  the integral constant
 * @param   Kd                  the derivative constant
 * @param   integralTolerance   the tolerance range for the integral to grow, in degrees
 * @param   settleTolerance     the tolerance range for the PID to be considered settled, in degrees
 * @param   settleTime          the amount of time the error must be within the settleTolerance before it is truly settled, in seconds
 * @param   minOutput           the minimum acceptable output, in volts
 * @param   maxOutput           the maximum acceptable output, in volts
 * 
 * @return  the time it takes for the PID to settle or time out
 */
float chassis::arcTo(vex::turnType direction, float radius, float heading, float timeout, float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput)
{
    PID arcPID = PID(Kp, Ki, Kd, integralTolerance, settleTolerance, settleTime, minOutput, maxOutput, 10);
    float multiplier = (radius - trackWidth/2) / (radius + trackWidth/2);

    float t = 0;
    if(direction == vex::turnType::right){
        while(!arcPID.isSettled() && t < timeout){
            float error = this->restrain(heading - this->Inertial->heading(vex::rotationUnits::deg), -180, 180);
            float output = arcPID.getOutput(error);

            this->Left->spin(vex::directionType::fwd, output, vex::voltageUnits::volt);
            this->Right->spin(vex::directionType::fwd, multiplier * output, vex::voltageUnits::volt);

            vex::task::sleep(10);
            t += 0.01;
        }
    }
    else{
        while(!arcPID.isSettled() && t < timeout){
            float error = this->restrain(this->Inertial->heading(vex::rotationUnits::deg), -180, 180);
            float output = arcPID.getOutput(error);

            this->Left->spin(vex::directionType::fwd, multiplier * output, vex::voltageUnits::volt);
            this->Right->spin(vex::directionType::fwd, output, vex::voltageUnits::volt);

            vex::task::sleep(10);
            t += 0.01;
        }
    }

    this->stopDrive(vex::brakeType::hold);

    return t;
}

/**
 * Sets the arc speed of the drivetrain in a specified direction
 * 
 * @param   direction   desired direction
 * @param   radius      desired radius
 * @param   speed       desired speed
 * @param   unit        speed's unit
 */
void chassis::setArcSpeed(vex::turnType direction, float radius, float speed, vex::velocityUnits unit)
{
    float multiplier = (radius - trackWidth/2) / (radius + trackWidth/2);
    if(direction == vex::turnType::right){
        this->Left->spin(vex::directionType::fwd, speed, unit);
        this->Right->spin(vex::directionType::fwd, multiplier * speed, unit);
    }
    else{
        this->Left->spin(vex::directionType::fwd, multiplier * speed, unit);
        this->Right->spin(vex::directionType::fwd, speed, unit);
    }
}

/**
 * Sets the arc speed of the drivetrain in a specified direction
 * 
 * @param   direction   desired direction
 * @param   radius      desired radius
 * @param   speed       desired speed
 * @param   unit        speed's unit
 */
void chassis::setArcSpeed(vex::turnType direction, float radius, float speed, vex::voltageUnits unit)
{
    float multiplier = (radius - trackWidth/2) / (radius + trackWidth/2);
    if(direction == vex::turnType::right){
        this->Left->spin(vex::directionType::fwd, speed, unit);
        this->Right->spin(vex::directionType::fwd, multiplier * speed, unit);
    }
    else{
        this->Left->spin(vex::directionType::fwd, multiplier * speed, unit);
        this->Right->spin(vex::directionType::fwd, speed, unit);
    }
}
