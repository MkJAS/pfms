#ifndef RECTANGLE_H
#define RECTANGLE_H

#include "shape.h"

class Rectangle : public Shape
{
public:
    Rectangle();
    Rectangle(double side);
    Rectangle(double width, double height);
    Rectangle(geometry_msgs::Point position, double width, double height);
    ~Rectangle();

    void setWidthHeight(double width, double height);
    
    void getWidthHeight(double& width, double& height);

    //!ADDITIONAL QUESTIONS TO CONSIDER
    // Consider if getArea() is a method that should exist in Rectangle?
    // Should all shapes be able to computer Area? Do all shapes have this attribute?
    // A design to enable this is covered in when we introduce polymorphism
    double getArea (void);

private:
    /**
     * @brief Function that updates the description based on the current member variables width and heigth
     */
    void updateDescription();

private:

    //!ADDITIONAL QUESTIONS TO CONSIDER
    // Why are these member varaibles in Rectangle, and not in shape?

    double width_; //!< width of rectangle
    double height_;//!< height of rectangle
};

#endif // RECTANGLE_H
