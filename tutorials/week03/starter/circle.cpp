#include "circle.h"
#include <math.h>

Circle::Circle()
{
    Circle(0);
}
Circle::Circle(double radius)
{
    radius_ = radius;
    area_ = radius*radius*M_PI;
    description_ = "Circle";
    setCentre(0,0);
}

double Circle::getArea()
{
    return area_;
}

bool Circle::checkIntercept(double x, double y)
{
    return std::pow(x-centreX_,2)+std::pow(y-centreY_,2)<std::pow(radius_,2);
}