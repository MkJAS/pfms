#include "rectangle.h"

Rectangle::Rectangle()
{
    Rectangle(0,0);
}

Rectangle::Rectangle(double width, double height)
{
    width_ = width;
    height_ = height;
    area_ = width*height;
    if (width == height)
        {
            description_ = "Square";
        }
    else
    {
        description_ = "Rectangle";
    }
    setCentre(0,0);    
}

double Rectangle::getArea()
{
    return area_;
}

bool Rectangle::checkIntercept(double x, double y)
{
    return (x<centreX_+(width_/2) && x>centreX_-(width_/2) && y<centreY_+(height_/2) && y>centreY_-(width_/2));

}