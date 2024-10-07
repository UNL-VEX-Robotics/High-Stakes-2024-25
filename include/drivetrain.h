#pragma once
#include "vex.h"
#include "pid.h"

class chassis{
private:

    /* ---------- Devices ---------- */
    vex::motor_group* Left;
    vex::motor_group* Right;

    vex::encoder* VerticalEncoder;
    vex::rotation* VerticalRotation;

    vex::inertial* Inertial;

    /* ---------- Functions ---------- */
    std::function<std::vector<float>()> getRobotPosition;

    float restrain(float num, float min, float max);
    float clamp(float num, float min, float max);
    float radToDeg(float rad);

    /* ---------- Data ---------- */
    enum verticalTracking{
        rotation,
        encoder,
        motorEncoder
    };

    verticalTracking trackingType;
    float trackWidth;
    float degreesToInches;

    /* ---------- PID Constants ---------- */
    struct {
        float Kp = 0;
        float Ki = 0;
        float Kd = 0;
        float integralTolerance = 0;
        float settleTolerance = 0;
        float settleTime = 0;
        float minOutput = 0;
        float maxOutput = 0;

        float headingKp = 0;
    }driveConstants;

    struct {
        float Kp = 0;
        float Ki = 0;
        float Kd = 0;
        float integralTolerance = 0;
        float settleTolerance = 0;
        float settleTime = 0;
        float minOutput = 0;
        float maxOutput = 0;
    }turnConstants;

    struct {
        float Kp = 0;
        float Ki = 0;
        float Kd = 0;
        float integralTolerance = 0;
        float settleTolerance = 0;
        float settleTime = 0;
        float minOutput = 0;
        float maxOutput = 0;
    }swingConstants;

    struct {
        float Kp = 0;
        float Ki = 0;
        float Kd = 0;
        float integralTolerance = 0;
        float settleTolerance = 0;
        float settleTime = 0;
        float minOutput = 0;
        float maxOutput = 0;
    }arcConstants;

public:
    /* --------- Constructor ---------- */
    chassis(std::function<std::vector<float>()> getRobotPosition, vex::motor_group* Left, vex::motor_group* Right, vex::inertial* Inertial, float trackWidth, float degreesToInches);
    chassis(std::function<std::vector<float>()> getRobotPosition, vex::motor_group* Left, vex::motor_group* Right, vex::inertial* Inertial, vex::encoder* VerticalEncoder, float trackWidth, float degreesToInches);
    chassis(std::function<std::vector<float>()> getRobotPosition, vex::motor_group* Left, vex::motor_group* Right, vex::inertial* Inertial, vex::rotation* VerticalRotation, float trackWidth, float degreesToInches);

    /* ---------- Tune PIDs ---------- */
    void setDriveConstants(float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput, float headingKp = 0);
    void setTurnConstants(float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput);
    void setSwingConstants(float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput);
    void setArcConstants(float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput);

    /* ---------- Drive ---------- */
    float driveFor(float distance);
    float driveFor(float distance, float timeout);
    float driveFor(float distance, float timeout, float heading);
    float driveFor(float distance, float timeout, float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput);
    float driveFor(float distance, float timeout, float heading, float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput, float headingKp);

    void setDriveSpeed(float speed, vex::velocityUnits unit);
    void setDriveSpeed(float speed, vex::voltageUnits unit);
    void stopDrive(vex::brakeType stopType = vex::brakeType::hold);

    float driveTo(float x, float y);
    float driveTo(float x, float y, float driveTimeout, float turnTimeout);
    float driveToReverse(float x, float y);
    float driveToReverse(float x, float y, float driveTimeout, float turnTimeout);

    /* ---------- Turn ---------- */
    float turnFor(float degrees);
    float turnFor(float degrees, float timeout);
    float turnFor(float degrees, float timeout, float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput);
    float turnTo(float heading);
    float turnTo(float heading, float timeout);
    float turnTo(float heading, float timeout, float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput);
    float turnToPosition(float x, float y);
    float turnToPosition(float x, float y, float timeout);
    float turnToPosition(float x, float y, float timeout, float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput);
    float turnToPositionReverse(float x, float y);
    float turnToPositionReverse(float x, float y, float timeout);
    float turnToPositionReverse(float x, float y, float timeout, float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput);

    void setTurnSpeed(float speed, vex::velocityUnits unit);
    void setTurnSpeed(float speed, vex::voltageUnits unit);

    /* ---------- Swing ---------- */
    float swingFor(vex::turnType direction, float degrees);
    float swingFor(vex::turnType direction, float degrees, float timeout);
    float swingFor(vex::turnType direction, float degrees, float timeout, float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput);
    float swingTo(vex::turnType direction, float heading);
    float swingTo(vex::turnType direction, float heading, float timeout);
    float swingTo(vex::turnType direction, float heading, float timeout, float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput);

    void setSwingSpeed(vex::turnType direction, float speed, vex::velocityUnits unit);
    void setSwingSpeed(vex::turnType direction, float speed, vex::voltageUnits unit);

    /* ---------- Arc ---------- */
    float arcFor(vex::turnType direction, float radius, float degrees);
    float arcFor(vex::turnType direction, float radius, float degrees, float timeout);
    float arcFor(vex::turnType direction, float radius, float degrees, float timeout, float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput);
    float arcTo(vex::turnType direction, float radius, float heading);
    float arcTo(vex::turnType direction, float radius, float heading, float timeout);
    float arcTo(vex::turnType direction, float radius, float heading, float timeout, float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput);

    void setArcSpeed(vex::turnType direction, float radius, float speed, vex::velocityUnits unit);
    void setArcSpeed(vex::turnType direction, float radius, float speed, vex::voltageUnits unit);
};