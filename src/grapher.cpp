/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       grapher.cpp                                               */
/*    Author:       Bryce Closman - UNLVEXU                                   */
/*    Created:      09/27/2024                                                */
/*    Description:  Graph Class Source Code                                   */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "grapher.h"


/**
 * Public function to set the bounds of the graph
 * 
 * @param   minX                the pixel x-location of the min value
 * @param   maxX                the pixel x-location of the max value
 * @param   minY                the pixel y-location of the min value
 * @param   maxY                the pixel y-location of the max value
 * @param   backgroundColor     the background color of the graph
 * @param   drawEmptyGraph      whether or not the method should immediatly draw the empty graph
 */
void Graph::setGraph(int minX, int maxX, int minY, int maxY, vex::color backgroundColor, bool drawEmptyGraph){
    this->outline.minXPixel = minX;
    this->outline.minYPixel = minY;
    this->outline.maxXPixel = maxX;
    this->outline.maxYPixel = maxY;
    this->backgroundColor = backgroundColor;

    if(drawEmptyGraph){
        this->Brain_Screen->setPenColor(this->outline.color);
        this->Brain_Screen->setFillColor(backgroundColor);
        this->Brain_Screen->setPenWidth(this->outline.thickness);
        this->Brain_Screen->drawRectangle(minX, 240-maxY, maxX-minX, maxY-minY);
    }
}


/**
 * Public function to set the outline color and thickness
 * 
 * @param   outlineColor    the color of the outline
 * @param   thickness       the thickness, in pixels, of the outline
 */
void Graph::setOutline(vex::color outlineColor, int thickness){
    this->outline.color = outlineColor;
    this->outline.thickness = thickness;
}


/**
 * Public function to set the scale of the graph
 * 
 * @param   xPixelsPerUnit  the number of pixels that represent one unit in the x direction
 * @param   yPixelsPerUnit  the number of pixels that represent one unit in the y direction
 * @param   minXValue       the minimum x value, in units
 * @param   minYValue       the minimum y value, in units
 */
void Graph::setScale(float xPixelsPerUnit, float yPixelsPerUnit, float minXValue, float minYValue){
    this->xPixelsPerUnit = xPixelsPerUnit;
    this->yPixelsPerUnit = yPixelsPerUnit;
    this->minXValue = minXValue;
    this->minYValue = minYValue;
}

/**
 * Public function to draw the data
 * will only draw if there is at least one point
 * 
 * @param   data            a std::vector<std::vector<float>> representing a series of points
 * @param   dataColor       the color the data should be
 * @param   penThickness    the thickness, in pixels, the data should be
 */
void Graph::drawData(std::vector<std::vector<float>> data, vex::color dataColor, int penThickness){
    this->Brain_Screen->setPenColor(dataColor);

    if(data.size() == 0) {}
    else if(data.size() == 1){
        this->Brain_Screen->drawCircle((data.at(0).at(0) - this->minXValue) * this->xPixelsPerUnit + this->outline.minXPixel, 240 - (this->outline.minYPixel + (data.at(0).at(1) - this->minYValue) * this->yPixelsPerUnit), penThickness);
    }
    else{
        this->Brain_Screen->setPenWidth(penThickness);

        for(int i = 0; i < data.size() - 1; i++){
            this->Brain_Screen->drawLine((data.at(i).at(0) - this->minXValue) * this->xPixelsPerUnit + this->outline.minXPixel, 240 - (this->outline.minYPixel + (data.at(i).at(1) - this->minYValue) * this->yPixelsPerUnit),\
                                            (data.at(i + 1).at(0) - this->minXValue) * this->xPixelsPerUnit + this->outline.minXPixel, 240 - (this->outline.minYPixel + (data.at(i + 1).at(1) - this->minYValue) * this->yPixelsPerUnit));
        }
    }
}


/**
 * Public function that will draw the outline of the graph
 */
void Graph::drawOutline(){
    this->Brain_Screen->setPenColor(this->outline.color);
    this->Brain_Screen->setFillColor(backgroundColor);
    this->Brain_Screen->setPenWidth(this->outline.thickness);
    this->Brain_Screen->drawRectangle(this->outline.minXPixel, 240-this->outline.maxYPixel, this->outline.maxXPixel-this->outline.minXPixel, this->outline.maxYPixel-this->outline.minYPixel);
}


/**
 * Public function that will automatically set the scale of the graph
 * 
 * @param   data    a std::vector<std::vector<float>> representing a series of points
 */
void Graph::autoScale(std::vector<std::vector<float>> data){
    float minX = data.at(0).at(0);
    float maxX = minX;
    float minY = data.at(0).at(1);
    float maxY = minY;

    for (int i = 1; i < data.size(); i++){
        float tempX = data.at(i).at(0);
        float tempY = data.at(i).at(1);

        if(tempX < minX) minX = tempX;
        if(tempX > maxX) maxX = tempX;
        if(tempY < minY) minY = tempY;
        if(tempY > maxY) maxY = tempY;
    }

    this->xPixelsPerUnit = (this->outline.maxXPixel - this->outline.minXPixel) / (maxX - minX);
    this->yPixelsPerUnit = (this->outline.maxYPixel - this->outline.minYPixel) / (maxY - minY);
    this->minXValue = minX;
    this->minYValue = minY;
}


/**
 * Constructor method
 * 
 * @param   Brain_Screen    a pointer to the brain's lcd screen
 */
Graph::Graph(vex::brain::lcd* Brain_Screen){
    this->Brain_Screen = Brain_Screen;
}