#include "triangle.h"

Triangle::Triangle():
    Triangle({0.0, 0.0}, 0.0, 0.0)
{
    description_="point";
}

Triangle::Triangle(double side):
    Triangle({0.0, 0.0}, side, side)
{

}


Triangle::Triangle(double width, double height):
    Triangle({0.0, 0.0}, width, height)
{

}

//! TODO - TASK 1: What do we have to initialise (type and sides
//! Check constructor of shape, check how to access type)
Triangle::Triangle(geometry_msgs::Point position, double width, double height):
    Shape(position,3), width_(width), height_(height)
{
    description_ = "isoc triangle";
    type_ = shape::Type::POLYGON;
}


Triangle::~Triangle(){

}

void Triangle::setHeightWidth(double width, double height)
{
    width_ = width;
    height_ = height;
}

void Triangle::getWidthHeight(double& width, double& height){
    width=width_;
    height = height_;
}

double Triangle::getArea()
{
    return width_ * height_ * 0.5;
}

