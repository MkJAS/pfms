#include "shape.h"

Shape::Shape(geometry_msgs::Point position, unsigned int sides):
    position_(position),
    sides_(sides),
    description_("unknown shape")
{
}

Shape::~Shape()
{
}

void Shape::setCentre(geometry_msgs::Point position)
{
    position_=position;
}

geometry_msgs::Point Shape::getCentre()
{
    return position_;
}

shape::Type Shape::getType()
{
    return type_;
}


std::string Shape::getDescription()
{
    return description_;
}

unsigned int Shape::getSides(){
    return sides_;
}
