/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       stanley.cpp                                               */
/*    Author:       Bryce Closman - UNLVEXU                                   */
/*    Created:      09/13/2024                                                */
/*    Description:  Stanley Class Source File                                 */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "stanley.h"

/**
 * Private function that returns the distance between two points
 * represented as a std::vector<float>
 * 
 * @param   x   a std::vector<float> containing a point in (x, y)
 * @param   y   a std::vector<float> containing a point in (x, y)
 * 
 * @return  the distance between x and y
 */
float Stanley::getDistance(std::vector<float> x, std::vector<float> y){
    return hypotf(x.at(0) - y.at(0), x.at(1) - y.at(1));
}


/**
 * Private function that returns the distance between a line defined by two points
 * and a point
 * 
 * @param   p0  the first point for a line
 * @param   p1  the second point for a line
 * @param   p   the individual point
 * 
 * @return  the distance from p to line p0, p1
 */
float Stanley::getDistance(std::vector<float> p0, std::vector<float> p1, std::vector<float> p){
    return (fabs((p1.at(1) - p0.at(1)) * p.at(0) - (p1.at(0) - p0.at(0)) * p.at(1) + p1.at(0) * p0.at(1) - p1.at(1) * p0.at(0))) / (hypotf(p1.at(1) - p0.at(1), p1.at(0) - p0.at(0)));
}


/**
 * Private function that restrains a number to a range
 * 
 * @param   num the number to be restrained
 * @param   min the minimum acceptable value
 * @param   max the maximum acceptable value
 * 
 * @return the restrained number
 */
float Stanley::restrain(float num, float min, float max){
    while(num > max) num -= (max - min);
    while(num < min) num += (max - min);
    return num;
}


/**
 * Private function that converts degrees to radians
 * 
 * @param   deg degrees
 * 
 * @return radians
 */
float Stanley::toRad(float deg){
    return deg * M_PI / 180;
}

/**
 * Private function that converts radians to degrees
 * 
 * @param   rad radians
 * 
 * @return degrees
 */
float Stanley::toDeg(float rad){
    return rad * 180 / M_PI;
}


/**
 * Private function that returns the point closest to the robot
 * 
 * @param   Path            a std::vector<std::vector<float>> representing a path
 * @param   currentPoint    the most recent point
 * 
 * @return  a vector containing (new current point, distance to robot, angle of tangent line)
 */
std::vector<float> Stanley::getPoint(std::vector<std::vector<float>> Path, int currentPoint){
    int numPoints = Path.size();
    float distance = INFINITY;
    float angle;
    float pointSave;
    std::vector<float> robotPosition = this->getRobotPosition();
    for (int i = currentPoint + 1; i < numPoints - 1; i++){
        float tempDistance = this->getDistance(Path.at(i), Path.at(i - 1), robotPosition);

        if(tempDistance < distance){
            distance = tempDistance;
            angle = atan2f(Path.at(i).at(1) - Path.at(i - 1).at(1), Path.at(i).at(0) - Path.at(i - 1).at(0));
            pointSave = (float)i;
        }
    }

    return {pointSave, distance, angle};
}

float Stanley::getTargetHeading(std::vector<std::vector<float>> Path, float kt, float currentVelocity){
    return 0;
}