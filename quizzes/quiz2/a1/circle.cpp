#include "circle.h"
#include <cmath>
#include <iostream>

Circle::Circle():
    Circle(0.0)
{
}

//! TODO - TASK 1: What do we have to initialise (type and sides
//! Check constructor of shape, check how to access type)
Circle::Circle(double radius):
    Shape({0.0, 0.0},1),radius_(radius)
{
    description_ = "circle";
    sides_ = 1;
    type_ = shape::Type::CIRCLE;
}

Circle::~Circle(){

}

void Circle::setRadius(double radius)
{
    radius_ = radius;
}

double Circle::getRadius()
{
    return radius_;
}


double Circle::getArea()
{
    return radius_ * radius_ * M_PI;
}

void Circle::getWidthHeight(double& width, double& height){
    width=radius_*2.0;
    height = radius_*2.0;
}
