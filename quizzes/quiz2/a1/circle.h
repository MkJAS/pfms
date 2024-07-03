#ifndef CIRCLE_H
#define CIRCLE_H

#include "shape.h"
#include "line.h"

class Circle : public Shape
{
public:
    Circle();
    Circle(double radius);
    ~Circle();
    void setRadius(double radius);
    double getRadius();
    double getArea ();
    void getWidthHeight(double& width, double& height);

private:
    double radius_; //!< width of triangle
};

#endif // CIRCLE_H
