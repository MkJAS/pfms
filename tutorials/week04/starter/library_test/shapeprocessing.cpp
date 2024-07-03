#include "shapeprocessing.h"
#include <iostream>

ShapeProcessing::ShapeProcessing(vector<Shape*> shape)
{
  shape_ = shape;
}

bool ShapeProcessing::checkIntersections(double x, double y){
  for(auto s:shape_)
  {
    (*s).checkIntercept(x,y);
  }

  return true;
}
