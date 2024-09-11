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

class odom{
    float verticalDistanceFromCenter;
    float horizontalDistanceFromCenter;
    float verticalInchesPerDegree;
    float horizontalInhcesPerDegree;

    vex::inertial *Inertial;

    vex::rotation *verticalRotation;
    vex::rotation *horizontalRotation;

    vex::encoder *verticalEncoder;
    vex::encoder *horizontalEncoder;

    bool usesRotation;

    bool isRunning = false;

    std::vector<float> robotPosition = {0, 0, 0};

    float degToRad(float deg);

    int updateRateMilliseconds;
public:
    void start();
    void stop();

    odom(vex::rotation &verticalRotation, vex::rotation &horizontalRotation, vex::inertial &Inertial, float verticalDistanceFromCenter, float verticalInchesPerDegree, float horizontalDistanceFromCenter, float horizontalInchesPerDegree, int updateRateMilliseconds);
    odom(vex::encoder &verticalEncoder, vex::encoder &horizontalEncoder, vex::inertial &Inertial, float verticalDistanceFromCenter, float verticalInchesPerDegree, float horizontalDistanceFromCenter, float horizontalInchesPerDegree, int updateRateMilliseconds);
    odom();

    std::vector<float> getPosition();
    float getX();
    float getY();
    float getHeading();

    void setPosition(float x, float y, float heading);
    void setX(float x);
    void setY(float y);
    void setHeading(float heading);
};