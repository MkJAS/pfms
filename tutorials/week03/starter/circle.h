#ifndef CIRCLE_H
#define CIRCLE_H
#include "shape.h"

class Circle : public Shape
{
public:
  Circle();
  Circle(double radius);
  double getArea();
  bool checkIntercept(double x, double y);

 private:
  double radius_; 
  double area_;

  
  
};

#endif // CIRCLE_H
