#include "analysis.h"
#include <cmath>
#include <memory>

#include <iostream>
#include "circle.h"
#include "rectangle.h"

using std::min;
using std::max;

Analysis::Analysis(std::vector<Shape*> shapes,Line line):
    shapes_(shapes),line_(line){

}

//! TODO - TASK 2: Implement below function
std::vector<int> Analysis::detectShapes(){

    std::vector<int> count(3,0); //creates a vector of 3 elements, each being a zero
    int numCirc = 0;
    int numTri = 0;
    int numRect = 0;
    unsigned int numsides = 0;

    for (unsigned i=0; i<shapes_.size(); i++ ) 
    {
        numsides = shapes_[i]->getSides();
        if (numsides == 1)
        {
            numCirc++;
        }
        else if (numsides == 3)
        {
            numTri++;
        }
        else if (numsides == 4)
        {
            numRect++;
        }
    }

    count[0] = numCirc;
    count[1] = numTri;
    count[2] = numRect;

    return count;

}


//! TODO - TASK 3 and 4: Implement below function
//! This function should call intersectsRectangle and intersectsTriangle
std::vector<bool> Analysis::intersectsLine() {

    std::vector<bool> intersects;
    for (auto s : shapes_) {
        std::cout << s->getType() << " " <<
                     s->getSides() << std::endl;
        if (s->getSides() == 4)
        {
            intersects.push_back(intersectsRectangle(s));
        }
        else if (s->getSides() == 3)
        {
            intersects.push_back(intersectsTriangle(s));
        }
    }
    return intersects;
}

//! TODO - TASK 3: Implement below function
bool Analysis::intersectsRectangle(Shape *shape) {

    std::cout << shape->getType() << std::endl;
    geometry_msgs::Point linept1{0,0};
    geometry_msgs::Point linept2{0,0};
    line_.getPoints(linept1,linept2);
    //pass 4 points
    //2 points of the line
    //2 points belonging to an edge of rectangle
    //need to find corners of rect
    geometry_msgs::Point centre = shape->getCentre();
    double width = 0, height = 0;
    shape->getWidthHeight(width,height);
    //we count Rect corners starting from top left going counter clockwise
    geometry_msgs::Point a{centre.x-(width/2),centre.y+(height/2)};
    geometry_msgs::Point b{centre.x-(width/2),centre.y-(height/2)};
    geometry_msgs::Point c{centre.x+(width/2),centre.y-(height/2)};
    geometry_msgs::Point d{centre.x+(width/2),centre.y+(height/2)};
    
    if (doIntersect(linept1,linept2,a,b)){return true;}
    if (doIntersect(linept1,linept2,b,c)){return true;}
    if (doIntersect(linept1,linept2,c,d)){return true;}
    if (doIntersect(linept1,linept2,a,d)){return true;}
    
    return false;

}

//! TODO - TASK 4: Implement below function
bool Analysis::intersectsTriangle(Shape* shape) {

    std::cout << shape->getType() << std::endl;
    geometry_msgs::Point linept1{0,0};
    geometry_msgs::Point linept2{0,0};
    line_.getPoints(linept1,linept2);
    //
    geometry_msgs::Point centre = shape->getCentre();
    double width = 0, height = 0;
    shape->getWidthHeight(width,height);
    //we count Tri corners starting from top going counter clockwise
    geometry_msgs::Point a{centre.x,centre.y+(height/2)};
    geometry_msgs::Point b{centre.x-(width/2),centre.y-(height/2)};
    geometry_msgs::Point c{centre.x+(width/2),centre.y-(height/2)};
    
    
    if (doIntersect(linept1,linept2,a,b)){return true;}
    if (doIntersect(linept1,linept2,b,c)){return true;}
    if (doIntersect(linept1,linept2,c,a)){return true;}
    
    return false;
}


//! THIS IS NOT PART OF THE QUIZ
bool Analysis::intersectsCircle(Shape* shape) {
    std::cout << shape->getType() << std::endl;
    return false;
}


// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool Analysis::onSegment(geometry_msgs::Point p, geometry_msgs::Point q, geometry_msgs::Point r)
{
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
        q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
       return true;

    return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int Analysis::orientation(geometry_msgs::Point p, geometry_msgs::Point q, geometry_msgs::Point r)
{
    // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
    // for details of below formula.
    int val = static_cast<int>(
                (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y)
                );

    if (val == 0) return 0;  // colinear

    return (val > 0)? 1: 2; // clock or counterclock wise
}

// The main function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool Analysis::doIntersect(geometry_msgs::Point p1, geometry_msgs::Point q1, geometry_msgs::Point p2, geometry_msgs::Point q2)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;

    // p1, q1 and q2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;

     // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false; // Doesn't fall in any of the above cases
}
