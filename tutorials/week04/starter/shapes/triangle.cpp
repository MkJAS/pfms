#include "triangle.h"

Triangle::Triangle(double width, double height):
    width_(width), height_(height)
{
    description_ = "isoc triangle";
}

void Triangle::setHeightWidth(double width, double height)
{
    width_ = width;
    height_ = height;
}

double Triangle::getArea()
{
    return width_ * height_ * 0.5;
}

//! Shoudl this work for triangle?
bool Triangle::checkIntercept(double x, double y){
      float ax = centreX_-(width_/2);
      float ay = centreY_-(height_/2);
      float bx = centreX_+(width_/2);
      float by = centreY_-(height_/2);
      float cy = centreY_+(height_/2);
      float cx = centreX_;

      bx -= ax;
      by -= ay;
      cx -= ax;
      cy -= ay;
      x -= ax;
      y -= ay;
      ax = 0;
      ay = 0;

      float d = bx*cy-cx*by;
      float wa = (x*(by-cy)+y*(cx-bx)+bx*cy-cx*by)/d;
      float wb = ((x*cy)-(y*cx))/d;
      float wc = ((y*bx)-(x*-by))/d;

      return (wa>0 && wb>0 && wc>0);


}
