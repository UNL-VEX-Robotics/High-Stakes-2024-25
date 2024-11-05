#include "ladybrown.h"

/**
 * Constructor method for the ladybrown class
 * 
 * @param motorGroup a pointer to the ladybrown motor group
 * @param downPosition the position of the motors when the ladybrown mech is down
 * @param readyPosition the position of the motors when the ladybrown mech is ready
 * @param storagePosition the position of the motors when the ladybrown mech is storing a ring
 * @param scorePosition the position of the motors when the ladybrown mech is scoring
 */
ladybrown::ladybrown(vex::motor_group *motorGroup, int downPosition, int readyPosition, int storagePosition, int scorePosition) :
    m_ladybrown_motors(motorGroup)
{
    ladybrown_positionValues[DOWN] = downPosition;
    ladybrown_positionValues[READY] = readyPosition;
    ladybrown_positionValues[STORAGE] = storagePosition;
    ladybrown_positionValues[SCORE] = scorePosition;
}

/**
 * Tunes the constants for the lady brown PID
 * 
 * @param kP proporitonal constant
 * @param kI integral constant
 * @param kD derivative constant
 * @param integralTolerance range for integral to be active
 * @param settleTolerance range for PID to be considered settled
 */
void ladybrown::setPIDConstants(float kP, float kI, float kD, float integralTolerance, float settleTolerance)
{
    m_PID = PID(kP, kI, kD, integralTolerance, settleTolerance, 30, -12, 12, 10);
}

/**
 * Gets the nearest position of the ladybrown mechanism
 * 
 * @return nearest position
 */
ladybrown::ladybrown_positions ladybrown::getNearestPosition()
{
    ladybrown_positions nearestPosition = DOWN;
    int minError = 2147483647;
    for (int i = 0; i < 4; i++)
    {
        int error = abs((int)(m_ladybrown_motors->position(vex::rotationUnits::deg)) - ladybrown_positionValues[i]);
        if(error < minError) 
        {
            minError = error;
            nearestPosition = (ladybrown_positions)i;
        }
    }

    return nearestPosition;
}

/**
 * Gets the current target position
 * 
 * @return the current target position
 */
ladybrown::ladybrown_positions ladybrown::getTargetPosition()
{
    return m_targetPosition;
}

/**
 * The main task of the ladybrown mechanism
 */
void ladybrown::ladybrown_task()
{
    m_runLadybrownTask = true;
    while(m_runLadybrownTask)
    {
        if(m_PID.isSettled()) m_ladybrown_motors->stop(vex::brakeType::hold);
        else {
            float output = m_PID.getOutput(ladybrown_positionValues[(int)m_targetPosition] - m_ladybrown_motors->position(vex::rotationUnits::deg));
            m_ladybrown_motors->spin(vex::directionType::fwd, output, vex::voltageUnits::volt);
        }

        vex::task::sleep(10);
    }
}

/**
 * Stops the main task of the ladybrown mechanism
 */
void ladybrown::stopTask()
{
    m_runLadybrownTask = false;
}

/**
 * Sets the target position of the ladybrown mechanism
 * 
 * @param newTarget the new target position
 */
void ladybrown::setTarget(ladybrown_positions newTarget)
{
    m_targetPosition = newTarget;
}

/**
 * Sets the current position of the ladybrown mechanism
 * 
 * @param position the current position
 */
void ladybrown::setCurrentPosition(int position)
{
    m_ladybrown_motors->setPosition(position, vex::rotationUnits::deg);
}
