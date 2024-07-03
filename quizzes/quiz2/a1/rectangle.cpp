#include "rectangle.h"
#include <cmath>

Rectangle::Rectangle():
    Rectangle({0.0, 0.0}, 0.0, 0.0)
{

}

Rectangle::Rectangle(double side):
    Rectangle({0.0, 0.0}, side, side)
{

}


Rectangle::Rectangle(double width, double height):
    Rectangle({0.0, 0.0}, width, height)
{

}

//! TODO - TASK 1: What do we have to initialise (type and sides
//! Check constructor of shape, check how to access type)
Rectangle::Rectangle(geometry_msgs::Point position, double width, double height):
    Shape(position,4), width_(width), height_(height)
{
    updateDescription();
    type_ = shape::Type::POLYGON;
}

Rectangle::~Rectangle(){

}


void Rectangle::setWidthHeight(double width, double height)
{
    //!NOTES 
    // This is a example of why you should not allow direct access to member variables (why they are private)
    // Given we have a function to set the member varaibles, we also can leverage this function to set any
    // other member variables required, of perform any other operations that are needed to be executed
    // (such as invoking other methods)

    width_ = width;
    height_ = height;

    updateDescription();
}

void Rectangle::getWidthHeight(double& width, double& height){
    width=width_;
    height = height_;
}


double Rectangle::getArea(void)
{
    return width_ * height_;
}


void Rectangle::updateDescription(){
    if (fabs(width_-height_)<1e-6) {
        description_ = "square";
    } else {
        description_ = "rectangle";
    }
}
