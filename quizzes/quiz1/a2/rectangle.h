#ifndef RECTANGLE_H
#define RECTANGLE_H

#include "shape.h"
#include <string>

//1) TASK
//Modify the file rectangle [rectangle](./a2/rectangle.h) so it inherits from the base class of shape [shape](./a2/shape.h)

class Rectangle : public Shape
{
public:
//    TASK 2
//    The `Rectangle` class already has a special member function.
//    Thanks to *Polymorphism* we can have functions with same name but different number of parameters.
//    Can you add to the `Rectangle` class another function that enables the `Rectangle` to on creation have `width` , `height` and `description` initialised with values supplied by user of the class.
//    You will need to add the declaration of this member function in the [header of Rectangle class](./a2/rectangle.h) as
//    well as implement this function in [implementation file of Rectangle class](./a2/rectangle.cpp).
    Rectangle(double width, double height, std::string description);

    Rectangle();
    /**
     * @brief Function that sets width and height
     * @param width in [m]
     * @param height in [m]
     */
    void setHeightWidth(double width, double height);

    //!ADDITIONAL QUESTIONS TO CONSIDER
    // Consider if getArea() is a method that should exist in Rectangle?
    // Should all shapes be able to computer Area? Do all shapes have this attribute?
    // A design to enable this is covered in when we introduce polymorphism
    /**
     * @brief Function that returns area
     * @return area in [m2]
     */
    double getArea (void);
private:

    //!ADDITIONAL QUESTIONS TO CONSIDER
    // Why are these member varaibles in Rectangle, and not in shape?
    double width_; //!< width of rectangle
    double height_;//!< height of rectangle
};

#endif // RECTANGLE_H
