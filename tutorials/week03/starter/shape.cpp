#include "shape.h"

Shape::Shape():
    description_("unknown shape")
{
}

void Shape::setCentre(double x, double y)
{
    centreX_= x;
    centreY_= y;
}

void Shape::offSetCentre(double x, double y)
{
    centreX_ += x;
    centreY_ += y;
}

// void Shape::getCentre(double& x, double& y)
// {
//     x = centreX_;
//     y = centreY_;
// }

std::string Shape::getDescription()
{
    return description_;
}