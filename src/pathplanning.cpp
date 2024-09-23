/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       pathplanning.cpp                                          */
/*    Author:       Bryce Closman - UNLVEXU                                   */
/*    Created:      09/23/2024                                                */
/*    Description:  Path Planning Class Source Code                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "pathplanning.h"


/**
 * Private function that returns whether a number is positive or negative
 * assumes 0 is positive
 * 
 * @param   num the number to find the sign of
 * 
 * @return  -1 if negative, 1 otherwise
 */
int pathPlanner::sgn(float num){
    if(num < 0) return -1;
    return 1;
}


/**
 * Private function that returns the x value at any given t value of a bezier curve
 * 0<=t<=1
 * 
 * @param   t   the t value of the bezier curve
 * @param   p0  the beginning point of the bezier curve
 * @param   p1  the control point of the bezier curve
 * @param   p2  the ending point of the bezier curve
 * 
 * @return  the X value
 */
float pathPlanner::getX(float t, std::vector<float> p0, std::vector<float> p1, std::vector<float> p2){
    return p1.at(0) + powf(1 - t, 2) * (p0.at(0) - p1.at(0)) + powf(t, 2) * (p2.at(0) - p1.at(0));
}


/**
 * Private function that returns the first derivative of the x value at any given t value
 * 0<=t<=1
 * 
 * @param   t   the t value of the bezier curve
 * @param   p0  the beginning point of the bezier curve
 * @param   p1  the control point of the bezier curve
 * @param   p2  the ending point of the bezier curve
 * 
 * @return  the first derivative of the x value
 */
float pathPlanner::getX_prime(float t, std::vector<float> p0, std::vector<float> p1, std::vector<float> p2){
    return 2 * (1 - t) * (p1.at(0) - p0.at(0)) + 2 * t * (p2.at(0) - p1.at(0));
}


/**
 * Private function that returns the second derivative of the x value at any given t value
 * 0<=t<=1
 * 
 * @param   t   the t value of the bezier curve
 * @param   p0  the beginning point of the bezier curve
 * @param   p1  the control point of the bezier curve
 * @param   p2  the ending point of the bezier curve
 * 
 * @return  the second derivative of the x value
 */
float pathPlanner::getX_doublePrime(float t, std::vector<float> p0, std::vector<float> p1, std::vector<float> p2){
    return 2 * (p2.at(0) - (2 * p1.at(0)) + p0.at(0));
}


/**
 * Private function that returns the y value at any given t value of a bezier curve
 * 0<=t<=1
 * 
 * @param   t   the t value of the bezier curve
 * @param   p0  the beginning point of the bezier curve
 * @param   p1  the control point of the bezier curve
 * @param   p2  the ending point of the bezier curve
 * 
 * @return  the y value
 */
float pathPlanner::getY(float t, std::vector<float> p0, std::vector<float> p1, std::vector<float> p2){
    return p1.at(1) + powf(1 - t, 2) * (p0.at(1) - p1.at(1)) + powf(t, 2) * (p2.at(1) - p1.at(1));
}


/**
 * Private function that returns the first derivative of the y value at any given t value
 * 0<=t<=1
 * 
 * @param   t   the t value of the bezier curve
 * @param   p0  the beginning point of the bezier curve
 * @param   p1  the control point of the bezier curve
 * @param   p2  the ending point of the bezier curve
 * 
 * @return  the first derivative of the y value
 */
float pathPlanner::getY_prime(float t, std::vector<float> p0, std::vector<float> p1, std::vector<float> p2){
    return 2 * (1 - t) * (p1.at(1) - p0.at(1)) + 2 * t * (p2.at(1) - p1.at(1));
}


/**
 * Private function that returns the second derivative of the y value at any given t value
 * 0<=t<=1
 * 
 * @param   t   the t value of the bezier curve
 * @param   p0  the beginning point of the bezier curve
 * @param   p1  the control point of the bezier curve
 * @param   p2  the ending point of the bezier curve
 * 
 * @return  the second derivative of the y value
 */
float pathPlanner::getY_doublePrime(float t, std::vector<float> p0, std::vector<float> p1, std::vector<float> p2){
    return 2 * (p2.at(1) - (2 * p1.at(1)) + p0.at(1));
}


/**
 * Private function that returns the arc length between two points on a bezier curve
 * 0<=t<=1
 * 
 * @param   t0  the t value of the first point on the bezier curve
 * @param   t1  the t value of the second point on the bezier curve
 * @param   p0  the beginning point of the bezier curve
 * @param   p1  the control point of the bezier curve
 * @param   p2  the ending point of the bezier curve
 * 
 * @return  the arc length between 2 points
 */
float pathPlanner::getDistance(float t0, float t1, std::vector<float> p0, std::vector<float> p1, std::vector<float> p2){
    float a = powf((p2.at(0) - p1.at(0)) - (p1.at(0) - p0.at(0)), 2) + powf((p2.at(1) - p1.at(1)) - (p1.at(1) - p0.at(1)), 2);
    float b = (p1.at(0) - p0.at(0)) * ((p2.at(0) - p1.at(0)) - (p1.at(0) - p0.at(0))) + (p1.at(1) - p0.at(1)) * ((p2.at(1) - p1.at(1)) - (p1.at(1) - p0.at(1)));
    float c = powf(p1.at(0) - p0.at(0), 2) + powf(p1.at(1) - p0.at(1), 2);

    float s1 = (((a * c - powf(b, 2)) * asinhf((a * t1 + b) / sqrtf(a * c - powf(b, 2)))) / (2 * powf(a, 1.5))) + (((a * t1 + b) * sqrtf(t1 * (a * t1 + 2 * b) + c)) / (2 * a));
    float s0 = (((a * c - powf(b, 2)) * asinhf((a * t0 + b) / sqrtf(a * c - powf(b, 2)))) / (2 * powf(a, 1.5))) + (((a * t0 + b) * sqrtf(t0 * (a * t0 + 2 * b) + c)) / (2 * a));

    return s1 - s0;
}


/**
 * Private function that calculates the radius of the curve at any given t value
 * 0<=t<=1
 * 
 * @param   t   the t value of the bezier curve
 * @param   p0  the beginning point of the bezier curve
 * @param   p1  the control point of the bezier curve
 * @param   p2  the ending point of the bezier curve
 * 
 * @return  the radius of the curve at t
 */
float pathPlanner::getRadius(float t, std::vector<float> p0, std::vector<float> p1, std::vector<float> p2){
    float xPrime = this->getX_prime(t, p0, p1, p2);
    float xDoublePrime = this->getX_doublePrime(t, p0, p1, p2);
    float yPrime = this->getY_prime(t, p0, p1, p2);
    float yDoublePrime = this->getY_doublePrime(t, p0, p1, p2);

    if (((xPrime * yDoublePrime) - (yPrime * xDoublePrime)) == 0) return INFINITY;
    return -1 * (powf(powf(xPrime, 2) + powf(yPrime, 2), 1.5) / ((xPrime * yDoublePrime) - (yPrime * xDoublePrime)));
}


/**
 * Private function that calculates the target heading at any given t value
 * 0<=t<=1
 * 
 * @param   t   the t value of the bezier curve
 * @param   p0  the beginning point of the bezier curve
 * @param   p1  the control point of the bezier curve
 * @param   p2  the ending point of the bezier curve
 * 
 * @return  the target heading at t in radians
 */
float pathPlanner::getHeading(float t, std::vector<float> p0, std::vector<float> p1, std::vector<float> p2){
    float xPrime = this->getX_prime(t, p0, p1, p2);
    float yPrime = this->getY_prime(t, p0, p1, p2);

    return atan2f(xPrime, yPrime);
}


/**
 * Public function that calculates the path using quadratic bezier curves
 * The path then gets a motion profile based on the physical constraints of the robot
 * angular velocity is in rad/s, heading in rad
 * 
 * @param   controlPoints   a vector containing vectors of 3 points, represented as vectors (x, y)
 * 
 * @return  a Path with a motion porfile
 */
std::vector<pathPlanner::Point> pathPlanner::generatePath(std::vector<std::vector<std::vector<float>>> controlPoints){
    std::vector<pathPlanner::Point> Path;

    for (int i = 0; i < controlPoints.size(); i++) for (float t = 0; t < 1; t += 0.001) {
        pathPlanner::Point tempPoint;

        tempPoint.x = this->getX(t, controlPoints.at(i).at(0), controlPoints.at(i).at(1), controlPoints.at(i).at(2));
        tempPoint.y = this->getY(t, controlPoints.at(i).at(0), controlPoints.at(i).at(1), controlPoints.at(i).at(2));
        tempPoint.t = t;
        tempPoint.radius = this->getRadius(t, controlPoints.at(i).at(0), controlPoints.at(i).at(1), controlPoints.at(i).at(2));
        tempPoint.heading = this->getHeading(t, controlPoints.at(i).at(0), controlPoints.at(i).at(1), controlPoints.at(i).at(2));
        
        if (t == 0 && i > 0) tempPoint.arcLength = this->getDistance(0.999, 1, controlPoints.at(i - 1).at(0), controlPoints.at(i - 1).at(1), controlPoints.at(i - 1).at(2));
        else if (t == 0) tempPoint.arcLength = 0;
        else if (Path.at(Path.size() - 1).t > tempPoint.t) tempPoint.arcLength = this->getDistance(0.999, 1, controlPoints.at(i - 1).at(0), controlPoints.at(i - 1).at(1), controlPoints.at(i - 1).at(2)) + getDistance(0, t, controlPoints.at(i).at(0), controlPoints.at(i).at(1), controlPoints.at(i).at(2));
        else tempPoint.arcLength = this->getDistance(Path.at(Path.size() - 1).t, t, controlPoints.at(i).at(0), controlPoints.at(i).at(1), controlPoints.at(i).at(2));

        if (t == 0 && i == 0) Path.push_back(tempPoint);
        else if (tempPoint.arcLength > this->desiredPointDistance) Path.push_back(tempPoint);
    }

    for (int i = 0; i < Path.size(); i++) {
        if (Path.at(i).radius == INFINITY) {
            Path.at(i).angularVelocity = 0;
            Path.at(i).velocity = this->maxVelocity;
        }
        else {
            Path.at(i).angularVelocity = sgn(Path.at(i).radius) * (this->maxVelocity / (fabs(Path.at(i).radius) + this->trackWidth / 2));
            Path.at(i).velocity = Path.at(i).angularVelocity * Path.at(i).radius;
        }
    }

    Path.at(0).angularVelocity = 0; 
    Path.at(0).velocity = 0; 

    for (int i = 1; i < Path.size(); i++) {
        float tempVelocity = sqrtf(powf(Path.at(i - 1).velocity, 2) + 2 * this->maxAcceleration * \
                                Path.at(i).arcLength);

        if (tempVelocity < Path.at(i).velocity) {
            Path.at(i).velocity = tempVelocity;
            if (Path.at(i).radius == INFINITY) Path.at(i).angularVelocity = 0;
            else Path.at(i).angularVelocity = tempVelocity / Path.at(i).radius;
        }
    }

    Path.at(Path.size() - 1).angularVelocity = 0; 
    Path.at(Path.size() - 1).velocity = 0; 

    for (int i = Path.size() - 2; i >= 0; i--) {
        float tempVelocity = sqrtf(powf(Path.at(i + 1).velocity, 2) + 2 * this->maxAcceleration * \
            Path.at(i + 1).arcLength);

        if (tempVelocity < Path.at(i).velocity) { 
            Path.at(i).velocity = tempVelocity; 
            if (Path.at(i).radius == INFINITY) Path.at(i).angularVelocity = 0;
            else Path.at(i).angularVelocity = tempVelocity / Path.at(i).radius;
        }
    }

    Path.at(0).time = 0;

    for (int i = 1; i < Path.size(); i++) {
        float tempTime = (2 * Path.at(i).arcLength) / (Path.at(i).velocity + Path.at(i - 1).velocity);
        Path.at(i).time = tempTime + Path.at(i - 1).time;
    }

    return Path;
}


/**
 * Constructor method for the pathPlanner class
 * UNITS are the same unit used within your coordinate system
 * 
 * @param   maxVelocity             the maximum velocity of the robot, in UNITS/s
 * @param   maxAcceleration         the maximum acceleration of the robot, in UNITS/s/s
 * @param   trackWidth              the track width of the robot, in UNITS
 * @param   desiredPointDistance    the desired distance between 2 points in the path, in UNITS
 */
pathPlanner::pathPlanner(float maxVelocity, float maxAcceleration, float trackWidth, float desiredPointDistance){
    this->maxVelocity = maxVelocity;
    this->maxAcceleration = maxAcceleration;
    this->trackWidth = trackWidth;
    this->desiredPointDistance = desiredPointDistance;
}