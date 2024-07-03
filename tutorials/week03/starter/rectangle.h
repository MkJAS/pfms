#ifndef RECTANGLE_H
#define RECTANGLE_H
#include "shape.h"

class Rectangle : public Shape
{
public:
    Rectangle();
    Rectangle(double width, double height);
    double getArea();
    bool checkIntercept(double x, double y);

private:
    double width_;
    double height_;
    double area_;
};

#endif