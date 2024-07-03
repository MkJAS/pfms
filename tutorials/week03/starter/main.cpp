#include "shape.h"
#include "rectangle.h"
#include "circle.h"
#include <iostream>


int main ()
{
    Circle circle1(3);
    circle1.offSetCentre(2,2);
    Rectangle rectangle1(2,3);
    rectangle1.offSetCentre(1,2);
    
    std::cout<<std::boolalpha<<circle1.checkIntercept(3,2)<<std::endl;
    std::cout<<std::boolalpha<<rectangle1.checkIntercept(3,2)<<std::endl;
}