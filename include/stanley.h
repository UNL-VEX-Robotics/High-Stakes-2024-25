/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       stanley.h                                                 */
/*    Author:       Bryce Closman - UNLVEXU                                   */
/*    Created:      09/13/2024                                                */
/*    Description:  Stanley Class header                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#pragma once
#include "vex.h"

class Stanley{
    std::function<std::vector<float>()> getRobotPosition;

    float getDistance(std::vector<float> x, std::vector<float> y);
    float getDistance(std::vector<float> p0, std::vector<float> p1, std::vector<float> p);
    float restrain(float num, float min, float max);
    float toRad(float deg);
    float toDeg(float rad);
    std::vector<float> getPoint(std::vector<std::vector<float>> Path, int currentPoint);
public:
    float getTargetHeading(std::vector<std::vector<float>> Path, float kt, float currentVelocity);
};