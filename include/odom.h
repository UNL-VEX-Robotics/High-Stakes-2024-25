/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       odom.h                                                    */
/*    Author:       Bryce Closman - UNLVEXU                                   */
/*    Created:      08/09/2024                                                */
/*    Description:  Odometry Class header                                     */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#pragma once
#include "vex.h"

class odometry{
public:
    class odometry_pod{
    public:

        enum sensorType
        {
            V5_MOTOR_ENCODER,
            V5_ROTATION,
            OS_ENCODER,
            NONE
        };

        enum wheelOrientation
        {
            VERTICAL = 0,
            PARALLEL = 0,
            HORIZONTAL = 1,
            PERPENDICULAR = 1
        };

    private:
        sensorType m_sensor;
        wheelOrientation m_orientation;

        vex::encoder* m_encoder;
        vex::rotation* m_rotation;
        vex::motor* m_motor;

        float m_distanceFromTrackingCenter;
        float m_inchesPerDegree;

    public:
        odometry_pod();
        odometry_pod(wheelOrientation orientation, vex::encoder* encoder, float distanceFromTrackingCenter, float inchesPerDegree);
        odometry_pod(wheelOrientation orientation, vex::rotation* rotation, float distanceFromTrackingCenter, float inchesPerDegree);
        odometry_pod(wheelOrientation orientation, vex::motor* motor, float distanceFromTrackingCenter, float inchesPerDegree);

        float getDistanceFromTrackingCenter();
        float getPosition();
    };

private:
    float g_xPosition;
    float g_yPosition;

    odometry_pod m_verticalTrackingWheel;
    odometry_pod m_horizontalTrackingWheel;
    vex::inertial* m_Inertial;

    bool m_tracking = false;

    float convertToRadians(float degrees);

    float getVerticalChange(float& previousVerticalPosition);
    float getHorizontalChange(float& previousHorizontalPosition);
    float getChangeInRotation(float& previousRotation);

public:
    void startTracking(float initialX, float initialY, float initialHeading);
    void stopTracking();

    std::vector<float> getPosition();

    odometry(odometry_pod verticalTrackingWheel, odometry_pod horizontalTrackingWheel, vex::inertial* Inertial);
};