/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       purepursuit.h                                             */
/*    Author:       Bryce Closman - UNLVEXU                                   */
/*    Created:      09/16/2024                                                */
/*    Description:  Pure Pursuit Class header                                 */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#pragma once
#include "vex.h"
#include "pathplanning.h"

class ramsete{
    float tolerance;
    float beta;
    float zeta;
    float gearRatio;
    float wheelDiameter;
    float trackWidth;

    std::vector<pathPlanner::Point> Path;
    int currentPoint;

    std::function<std::vector<float>()> getRobotPosition;
    vex::motor_group *Left;
    vex::motor_group *Right;

    float getDistance(std::vector<float> p0, std::vector<float> p1);
    float toRad(float deg);
    float restrain(float num, float min, float max);

    int getTargetPoint(std::vector<float> robotPosition, std::vector<pathPlanner::Point> Path, int currentTarget);

    std::vector<float> getLocalError(std::vector<float> robotPosition, pathPlanner::Point targetPoint);


public:
    void runPath(std::vector<pathPlanner::Point> Path);
    void setConstants(float beta, float zeta);

    pathPlanner::Point getCurrentTarget();

    ramsete::ramsete(std::function<std::vector<float>()> getRobotPosition, vex::motor_group* Left, vex::motor_group* Right, float trackWidth, float wheelDiameter, float gearRatio, float tolerance);
};