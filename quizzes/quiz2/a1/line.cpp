#include "line.h"

Line::Line(geometry_msgs::Point pt1, geometry_msgs::Point pt2):
    pt1_(pt1), pt2_(pt2)
{
    fromPoints(pt1, pt2);
}

void Line::fromPoints(geometry_msgs::Point pt1, geometry_msgs::Point pt2)
{
    gradient_ = (pt2.y - pt1.y) / (pt2.x - pt1.x);
    y_intercept_ = pt1.y - gradient_ * pt1.x;
}

void Line::setGradient(double gradient)
{
    gradient_ = gradient;
}

void Line::setYIntercept(double y_intercept)
{
    y_intercept_ = y_intercept;
}

bool Line::pointAboveLine(geometry_msgs::Point pt)
{
    double line_y = gradient_ * pt.x + y_intercept_;
    return pt.y > line_y;
}

double Line::getGradient() {
    return gradient_;
}

double Line::getYIntercept() {
    return y_intercept_;
}

void Line::getPoints(geometry_msgs::Point& pt1, geometry_msgs::Point& pt2){
    pt1=pt1_;
    pt2=pt2_;
}
