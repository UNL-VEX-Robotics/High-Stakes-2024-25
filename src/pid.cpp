/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       pid.cpp                                                   */
/*    Author:       Bryce Closman - UNLVEXU                                   */
/*    Created:      08/15/2024                                                */
/*    Description:  PID Class source file                                     */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "pid.h"

/**
 * Creates a PID object with the specified constants
 * 
 * @param   Kp                  proportional constant
 * @param   Ki                  integral constant
 * @param   Kd                  derivative constant
 * @param   integralTolerance   error tolerance for the integral to be increased
 * @param   settleTolerance     error tolerance to be considered settled
 * @param   settleTime          time settled for the PID to end
 * @param   minOutput           minimum output
 * @param   maxOutput           maximum output
 * @param   cycleTime           cycle time in mS
 */
PID::PID(float Kp, float Ki, float Kd, float integralTolerance, float settleTolerance, \
    float settleTime, float minOutput, float maxOutput, int cycleTime){

    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->integralTolerance = integralTolerance;
    this->settleTolerance = settleTolerance;
    this->settleTime = settleTime;
    this->minOutput = minOutput;
    this->maxOutput = maxOutput;
    this->cycleTime = cycleTime;
}

/**
 * Sets the constants of a PID object
 * 
 * @param   Kp                  proportional constant
 * @param   Ki                  integral constant
 * @param   Kd                  derivative constant
 * @param   integralTolerance   error tolerance for the integral to be increased
 * @param   settleTolerance     error tolerance to be considered settled
 * @param   settleTime          time settled for the PID to end
 * @param   minOutput           minimum output
 * @param   maxOutput           maximum output
 * @param   cycleTime           cycle time in mS
 */
void PID::setConstants(float Kp, float Ki, float Kd, float integralTolerance, \
    float settleTolerance, float settleTime, float minOutput, float maxOutput, int cycleTime){

    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->integralTolerance = integralTolerance;
    this->settleTolerance = settleTolerance;
    this->settleTime = settleTime;
    this->minOutput = minOutput;
    this->maxOutput = maxOutput;
    this->cycleTime = cycleTime;
}

/**
 * Updates the PID and gives the output
 * 
 * @param   error           the current error
 * @param   stopIOvershoot  prevents I from causing overshoots
 * 
 * @return  the PID output
 */
float PID::getOutput(float error, bool stopIOvershoot){
    if((error * this->previousError < 0 && stopIOvershoot) || \
        fabs(error) > this->integralTolerance) this->integral = 0;
    else this->integral += error;

    float output = this->Kp * error + this->Ki * this->integral + this->Kd * (error - this->previousError);

    this->previousError = error;

    if(fabs(error) < this->settleTolerance) this->timeSettled += this->cycleTime;
    else this->timeSettled = 0;

    if(output > this->maxOutput) output = this->maxOutput;
    if(output < this->minOutput) output = this->minOutput;

    return output;
}

/**
 * Determines if the PID is settled
 * 
 * @return true if settled
 */
bool PID::isSettled(){
    return (this->settleTime <= this->timeSettled);
}