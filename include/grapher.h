/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       grapher.h                                                 */
/*    Author:       Bryce Closman - UNLVEXU                                   */
/*    Created:      09/27/2024                                                */
/*    Description:  Graph Class Header File                                   */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#pragma once
#include "vex.h"

class Graph{
    vex::brain::lcd* Brain_Screen;

    float xPixelsPerUnit;
    float yPixelsPerUnit;    
    float minXValue;
    float minYValue;

    vex::color backgroundColor;

    struct Graph_Outline{
        vex::color color;
        int thickness;

        int minXPixel;
        int minYPixel;
        int maxXPixel;
        int maxYPixel;
    } outline;
    
public:
    void setGraph(int minX, int maxX, int minY, int maxY, vex::color backgroundColor, bool drawEmptyGraph = false); 
    void setOutline(vex::color outlineColor, int outlineThickness);
    void setScale(float xPixelsPerUnit, float yPixelsPerUnit, float minXValue, float minYValue);
    void drawData(std::vector<std::vector<float>> data, vex::color dataColor, int penThickness);
    void drawOutline();
    void autoScale(std::vector<std::vector<float>> data);

    Graph(vex::brain::lcd* Brain_Screen);
};