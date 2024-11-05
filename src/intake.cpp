#include "intake.h"

/**
 * Constructor method for the intake class
 * 
 * @param intakeMotors a pointer to the intake motor group
 * @param ringSensor a pointer to the optical sensor on the intake
 */
intake::intake(vex::motor_group *intakeMotors, vex::optical* ringSensor, int ringEjectPosition, ladybrown* ladybrown) :
    m_ladybrown(ladybrown),
    m_Intake(intakeMotors),
    m_Optical(ringSensor),    
    m_currentSpeed(0),
    m_ringEjectPosition(ringEjectPosition)    
{  
}

/**
 * The main loop for the intake, accessible in bot autonomous and driver control
 */
void intake::intake_task()
{
    m_runIntakeTask = true;
    m_Optical->integrationTime(5);

    bool ejectRing = false;
    bool adjustedLadybrown = false;
    ladybrown::ladybrown_positions currentTarget;
    while(m_runIntakeTask)
    {
        currentTarget = m_ladybrown->getTargetPosition();
        if(m_enableColorSort)
        {
            if(m_Optical->isNearObject())
            {
                m_Optical->setLightPower(50, vex::percentUnits::pct);
                if((m_isRed && m_Optical->color() == vex::color::red) || (!m_isRed && m_Optical->color() == vex::color::blue)) ejectRing = true;
            }
            else m_Optical->setLight(vex::ledState::off);

            if(ejectRing)
            {
                adjustedLadybrown = true;
                if(currentTarget == ladybrown::ladybrown_positions::READY)
                {
                    switch (m_ladybrown->getNearestPosition())
                    {
                        case ladybrown::ladybrown_positions::SCORE:
                        case ladybrown::ladybrown_positions::STORAGE:
                            m_ladybrown->setTarget(ladybrown::ladybrown_positions::STORAGE);
                            break;
                        case ladybrown::ladybrown_positions::READY:
                        case ladybrown::ladybrown_positions::DOWN:
                            m_ladybrown->setTarget(ladybrown::ladybrown_positions::DOWN);
                            break;
                    }
                }

                if (abs(((int)m_Intake->position(vex::rotationUnits::deg) % m_ringEjectPosition) - m_ringEjectPosition) < 35)
                {
                    m_Intake->spin(vex::directionType::rev, 100, vex::percentUnits::pct);
                    vex::task::sleep(250);
                    ejectRing = false;

                    if(adjustedLadybrown)
                    {
                        adjustedLadybrown = false;
                        m_ladybrown->setTarget(ladybrown::ladybrown_positions::READY);
                    }
                }
            }
        }

        if(m_currentSpeed == 0) m_Intake->stop(m_stoppingType);
        else m_Intake->spin(vex::directionType::fwd, m_currentSpeed, vex::percentUnits::pct);
        
        vex::task::sleep(5);
    }
}

/**
 * Sets the speed of the intake
 * 
 * @param speed speed, in percent
 */
void intake::setSpeed(int8_t speed)
{
    m_currentSpeed = speed;
}

/**
 * Sets the brake type of the intake
 * 
 * @param stoppingType the new brake type
 */
void intake::setBrakeType(vex::brakeType stoppingType)
{
    m_stoppingType = stoppingType;
}

/**
 * enables/disables the color sort
 * 
 * @return the new color sort value
 */
bool intake::setColorSort(bool enable)
{
    m_enableColorSort = enable;
    return enable;
}

/**
 * Sets the color of the intake class
 * 
 * @param isRed bool whether the alliance is red
 */
void intake::setColor(bool isRed)
{
    m_isRed = isRed;
}
