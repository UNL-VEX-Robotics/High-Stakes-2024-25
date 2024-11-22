#pragma once
#include "vex.h"
#include "pid.h"

class ladybrown{
public:
    enum ladybrown_positions
    {
        DOWN = 0,
        READY = 1,
        STORAGE = 2,
        SCORE = 3
    };

private:
    vex::motor_group* m_ladybrown_motors;
    
    int ladybrown_positionValues[4];

    struct {
        float Kp = 0;
        float Ki = 0;
        float Kd = 0;
        float integralTolerance = 0;
        float settleTolerance = 0;
        float settleTime = 0;
        float minOutput = 0;
        float maxOutput = 0;
    }PID_constants;

    ladybrown_positions m_targetPosition = READY;
    bool m_runLadybrownTask;

public:
    ladybrown(vex::motor_group* motorGroup, int downPosition, int readyPosition, int storagePosition, int scorePosition);
    void setPIDConstants(float kP, float kI, float kD, float integralTolerance, float settleTolerance);
    ladybrown_positions getNearestPosition();
    ladybrown_positions getTargetPosition();

    void ladybrown_task();
    void stopTask();
    void setTarget(ladybrown_positions newTarget);
    void setCurrentPosition(int position);
};

