/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       pathplanning.h                                            */
/*    Author:       Bryce Closman - UNLVEXU                                   */
/*    Created:      09/23/2024                                                */
/*    Description:  Path Planning Class header                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#pragma once
#include "vex.h"

class pathPlanner {
public:
    struct Point{
        float x;
        float y;
        float heading;
        float t;
        float arcLength;
        float radius;
        float angularVelocity;
        float velocity;
        float time;
    };

private:
    float maxVelocity;
    float maxAcceleration;
    float trackWidth;
    float desiredPointDistance;
    
    int sgn(float num);

    float getX(float t, std::vector<float> p0, std::vector<float> p1, std::vector<float> p2);
    float getX_prime(float t, std::vector<float> p0, std::vector<float> p1, std::vector<float> p2);
    float getX_doublePrime(float t, std::vector<float> p0, std::vector<float> p1, std::vector<float> p2);
    
    float getY(float t, std::vector<float> p0, std::vector<float> p1, std::vector<float> p2);
    float getY_prime(float t, std::vector<float> p0, std::vector<float> p1, std::vector<float> p2);
    float getY_doublePrime(float t, std::vector<float> p0, std::vector<float> p1, std::vector<float> p2);

    float getDistance(float t0, float t1, std::vector<float> p0, std::vector<float> p1, std::vector<float> p2);
    float getRadius(float t, std::vector<float> p0, std::vector<float> p1, std::vector<float> p2); 
    float getHeading(float t, std::vector<float> p0, std::vector<float> p1, std::vector<float> p2);

public:
    std::vector<pathPlanner::Point> generatePath(std::vector < std::vector<std::vector<float>>> controlPoints);

    pathPlanner(float maxVelocity, float maxAcceleration, float trackWidth, float desiredPointDistance);
};