#ifndef LINE_H
#define LINE_H

#include "geometry_msgs.h"

class Line
{
public:
    Line(geometry_msgs::Point pt1, geometry_msgs::Point pt2 );
    bool pointAboveLine(geometry_msgs::Point pt);
    double getGradient();
    double getYIntercept();
    void getPoints(geometry_msgs::Point& pt1, geometry_msgs::Point& pt2);

private:
    void fromPoints(geometry_msgs::Point pt1, geometry_msgs::Point pt2);
    void setGradient(double gradient);
    void setYIntercept(double y_intercept);

private:
    double gradient_;
    double y_intercept_;
    geometry_msgs::Point pt1_;
    geometry_msgs::Point pt2_;
};

#endif // LINE_H
