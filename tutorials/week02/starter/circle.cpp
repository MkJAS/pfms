#include "circle.h"
#include <math.h>


Circle::Circle(double radius):radius_(radius)
{
    setperimeter(radius);
    setarea(radius);
}

void Circle::setradius(double radius)
{
    radius_ = radius;
    
}

void Circle::setarea(double radius)
{
    area_ = radius*radius*M_PI;
}

double Circle::getarea()
{
    //setarea(radius_);
    return area_;
}

void Circle::setperimeter(double radius)
{
    perimeter_= 2*radius*M_PI;
}

double Circle::getperimeter()
{
    //setperimeter(radius_);
    return perimeter_;
}

// double CirclesArea(std::vector<Circle> CircVec)
// {
//     double total_area;
//     // int n;
//     // double area = CircVec[0].getarea();

//     for (std::vector<Circle>::iterator it = CircVec.begin() ; it != CircVec.end(); ++it)
//     {
//         total_area += (*it).getarea();
//     }

    
//    return total_area;
// }