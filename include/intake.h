#pragma once
#include "vex.h"
#include "ladybrown.h"

class intake{
private:
    ladybrown* m_ladybrown;

    vex::motor_group* m_Intake;
    vex::optical* m_Optical;
    int8_t m_currentSpeed;
    int m_ringEjectPosition;
    int m_ladybrownPosition;

    vex::brakeType m_stoppingType = vex::brakeType::coast;
    
    bool m_isRed;
    bool m_runIntakeTask;
    bool m_enableColorSort = false;
    bool m_useLadyBrown = false;

    int m_targetLadyBrownPosition;
    
public:
    intake(vex::motor_group* intakeMotors, vex::optical* ringSensor, int ringEjectPosition, ladybrown* ladybrown, int ladybrownPosition);

    void intake_task();
    void setSpeed(int8_t speed);
    void setBrakeType(vex::brakeType stoppingType);
    bool setColorSort(bool enable);
    void setColorSortOffset(int offset);
    void setColor(bool isRed);
    void setLadybrown(bool val);
    bool getLadybrown();
};