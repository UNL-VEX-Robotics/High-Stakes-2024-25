/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       pid.h                                                     */
/*    Author:       Bryce Closman - UNLVEXU                                   */
/*    Created:      08/15/2024                                                */
/*    Description:  PID Class header                                          */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#pragma once
#include "vex.h"

class PID{
    float Kp;
    float Ki;
    float Kd;
    float integralTolerance;
    float settleTolerance;
    float settleTime;
    float minOutput;
    float maxOutput;
    int cycleTime;

    float previousError = 0;
    float integral = 0;
    float timeSettled = 0;

public:
    PID();
    PID(float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput, int cycleTime);
    void setConstants(float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, float settleTime, float minOutput, float maxOutput, int cycleTime);
    float getOutput(float error, bool stopIOvershoot = true);
    bool isSettled();
};