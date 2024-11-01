#pragma once
#include "vex.h"

class intake{
private:
    vex::motor_group* m_Intake;
    vex::optical* m_Optical;
    int8_t m_currentSpeed;
    int m_ringEjectPosition;

    vex::brakeType m_stoppingType = vex::brakeType::coast;
    
    bool m_isRed;
    bool m_runIntakeTask;
    bool m_enableColorSort = false;

    std::function<int()> getLadyBrownPosition;
    int m_targetLadyBrownPosition;
    
public:
    intake(vex::motor_group* intakeMotors, vex::optical* ringSensor, int ringEjectPosition, std::function<int()> ladyBrownPosition, int ladyBrownTargetPosition);

    void intake_task();
    void setSpeed(int8_t speed);
    void setBrakeType(vex::brakeType stoppingType);
    bool setColorSort(bool enable);
    void setColor(vex::color allianceColor);
};