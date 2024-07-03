#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "shape.h"
#include "line.h"

class Triangle : public Shape
{
public:
    Triangle();
    Triangle(double side);
    Triangle(double width, double height);
    Triangle(geometry_msgs::Point position, double width, double height);
    ~Triangle();

    void setHeightWidth(double width, double height);
    void getWidthHeight(double& width, double& height);
    double getArea ();
    void setCentre(geometry_msgs::Point);

private:
    double width_; //!< width of triangle
    double height_;//!< height of triangle
};


#endif // TRIANGLE_H
