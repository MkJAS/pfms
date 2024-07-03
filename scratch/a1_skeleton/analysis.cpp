#include "analysis.h"
#include <cmath>
#include <memory>

#include <iostream>

using std::min;
using std::max;


geometry_msgs::Point PointofLineIntersection(geometry_msgs::Point A,geometry_msgs::Point B,geometry_msgs::Point C, 
geometry_msgs::Point D)
{
    // Line AB represented as a1x + b1y = c1
    double a1 = B.y - A.y;
    double b1 = A.x - B.x;
    double c1 = a1*(A.x) + b1*(A.y);
  
    // Line CD represented as a2x + b2y = c2
    double a2 = D.y - C.y;
    double b2 = C.x - D.x;
    double c2 = a2*(C.x)+ b2*(C.y);
  
    double determinant = a1*b2 - a2*b1;
  
    if (determinant == 0)
    {
        // The lines are parallel. As such they cannot intersect
        //so we return random, N/A values to stream line code
        return geometry_msgs::Point{1234,1234};
    }
    else
    {
        double x = (b2*c1 - b1*c2)/determinant;
        double y = (a1*c2 - a2*c1)/determinant;
        //since we are dealing with line segments, we must check to see if the calculated point
        //exists along the length of the sides and not on some point outside its bounds
        if(min (A.x, B.x) <= x && x <= max (A.x, B.x) && min (A.y,B.y) <= y && y <= max(A.y,B.y))
        {
            if(min (C.x, D.x) <= x && x <= max (C.x, D.x) && min (C.y,D.y)-0.0001 <= y && y <= 0.0001+max(C.y,D.y))
            {
               return geometry_msgs::Point{x,y}; 
            }
            
        }
        //if point does not lie on line disregard it
        else return geometry_msgs::Point{1234,1234};
        
    }
    return geometry_msgs::Point{1234,1234};
}

bool Checkallsides(geometry_msgs::Point a,geometry_msgs::Point b,geometry_msgs::Point c,geometry_msgs::Point a1,
geometry_msgs::Point b1,geometry_msgs::Point c1,std::vector<geometry_msgs::Point> &all_POI)
{
    bool intersect = false;
    geometry_msgs::Point intsct;
    intsct = PointofLineIntersection(a,b,a1,b1);
    if(intsct.x != 1234 && intsct.y != 1234)
    {
        all_POI.push_back(intsct);
        intersect = true;
    }
    intsct = PointofLineIntersection(a,b,a1,c1);
    if(intsct.x != 1234 && intsct.y != 1234)
    {
        all_POI.push_back(intsct);
        intersect = true;
    }
    intsct = PointofLineIntersection(a,b,c1,b1);
    if(intsct.x != 1234 && intsct.y != 1234)
    {
        all_POI.push_back(intsct);
        intersect = true;
    }
    intsct = PointofLineIntersection(a,c,a1,b1);
    if(intsct.x != 1234 && intsct.y != 1234)
    {
        all_POI.push_back(intsct);
        intersect = true;
    }
    intsct = PointofLineIntersection(a,c,a1,c1);
    if(intsct.x != 1234 && intsct.y != 1234)
    {
        all_POI.push_back(intsct);
        intersect = true;
    }
    intsct = PointofLineIntersection(a,c,c1,b1);
    if(intsct.x != 1234 && intsct.y != 1234)
    {
        all_POI.push_back(intsct);
        intersect = true;
    }
    intsct = PointofLineIntersection(c,b,a1,b1);
    if(intsct.x != 1234 && intsct.y != 1234)
    {
        all_POI.push_back(intsct);
        intersect = true;
    }
    intsct = PointofLineIntersection(c,b,a1,c1);
    if(intsct.x != 1234 && intsct.y != 1234)
    {
        all_POI.push_back(intsct);
        intersect = true;
    }
    intsct = PointofLineIntersection(c,b,c1,b1);
    if(intsct.x != 1234 && intsct.y != 1234)
    {
        all_POI.push_back(intsct);
        intersect = true;
    }
    return intersect;

}

bool isInsideSonar(geometry_msgs::Point A,geometry_msgs::Point B,
geometry_msgs::Point C,geometry_msgs::Point P)
{
    //set one of the sonar points as origin by minusing that point from the others
    B.x = B.x - A.x;
    B.y = B.y - A.y;
    C.x = C.x - A.x;
    C.y = C.y - A.y;
    P.x = P.x - A.x;
    P.y = P.y - A.y;
    A.x = 0;
    A.y = 0;
    //calculate the 3 weights
    double d = (A.x*B.y-A.y*B.x)+(B.x*C.y-B.y*C.x)+(C.x*A.y-C.y*A.x);
    double xa = (B.x*C.y-B.y*C.x)+(P.x*(B.y-C.y)-P.y*(B.x-C.x));
    double xb = (C.x*A.y-C.y*A.x)+(P.x*(C.y-A.y)-P.y*(C.x-A.x));
    double xc = (A.x*B.y-A.y*B.x)+(P.x*(A.y-B.y)-P.y*(A.x-B.x));
    double wa = xa/d;
    double wb = xb/d;
    double wc = xc/d;
    //a point is within the sonar if all weights are between 1 and 0
    return (0<wa&&wa<1 && 0<wb&&wb<1 && 0<wc&&wc<1);
}

bool LaserIntersect(geometry_msgs::Point laserStart, geometry_msgs::Point laserEnd,geometry_msgs::Point a,
geometry_msgs::Point b,geometry_msgs::Point c,geometry_msgs::Point d) 
{
    //simply check if the laser line passes through any of the cell sides
    if (doIntersect(laserStart,laserEnd,a,b)){return true;}
    if (doIntersect(laserStart,laserEnd,b,c)){return true;}
    if (doIntersect(laserStart,laserEnd,c,d)){return true;}
    if (doIntersect(laserStart,laserEnd,a,d)){return true;}
    
    return false;

}

bool SonarOccupied(geometry_msgs::Point sonarStart, geometry_msgs::Point sonarCorner,double theta,double sonarangle,
double radius,geometry_msgs::Point a, geometry_msgs::Point b,geometry_msgs::Point c,geometry_msgs::Point d)
{
    double angle_ = 0;                          //angle to corresponding point on curve
    std::vector<geometry_msgs::Point> curvexy; //vector of points along curve in cartesian coordinates
    geometry_msgs::Point point;
    for(int i=0;i<sonarangle+1;i++)         //sonar angle is the FOV/2 (10)
    {
        angle_ = theta+sonarangle+90;       //the angle of the point STARTING FROM THE MOST COUNTER CLOCKWISE POINT
        angle_ = angle_-(i*2);              //then iterate clockwise to the least CCW point from  positive x axis
        
        point.x = sonarStart.x + radius*cos(2*M_PI - (M_PI/180)*angle_);    //calculate point from sonar origing then add sonar offset to obtain point from global perspective
        point.y = sonarStart.y + radius*sin(M_PI - (M_PI/180)*angle_);
        curvexy.push_back(point);
        //std::cout<<point.x<<" "<<point.y<<std::endl;
        
    }
    for(auto it:curvexy) //for every point that has just been calculated check if its within cell bounds
    {
        if(it.x>a.x-0.05 && it.x<c.x+0.05 && it.y>b.y-0.05 && it.y<d.y+0.05) //0.05 added offset for marginal intersection
        {
            return true;            
        }
    }
    return false;
}

bool SonarFree(double theta, double radius,double sonarangle,geometry_msgs::Point sonarStart,
geometry_msgs::Point sonarCorner, geometry_msgs::Point a, geometry_msgs::Point b)
{
    if (doIntersect(sonarStart,sonarCorner,a,b)){return true;}
    a.x = a.x - sonarStart.x;
    a.y = a.y - sonarStart.y;
    sonarStart.x = 0;
    sonarStart.y = 0;
    double dist_X = sonarStart.x - a.x;
    double dist_Y = sonarStart.y - a.y;
    double dist = sqrt(pow(dist_X,2)+pow(dist_Y,2)); //calculate distance(radius) between sonar and cell corner
    double fov_start = 90-sonarangle+theta; //bounds of sonar FOV in deg from +x axis
    double fov_end = fov_start+20;          //bounds of sonar FOV
    //double a_dist = sqrt(pow(a.x,2)+pow(a.y,2));    //calculate distance of cell corner from origin
    double a_angle = atan2(a.y,a.x)*180/M_PI; //calculate angle line from cell corner to origin makes with +x axis
    if(a_angle<0) //if angle comes out to be greater to 360 due to calculation minus 360 to get applicable values for test
    {
        a_angle = a_angle + 360;
    }
    if(dist>radius) //if cell corner is further from sonar start than the sonar reading it CANNOT be within the sonar bounds
    {
        return false;
    }
    if(dist<radius)
    {
        if(a_angle>fov_start && a_angle<fov_end) //the angle made bt cell corner must be within the sonar FOV range
        {
            return true;
        }
        else return false;
    }
}

// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool onSegment(geometry_msgs::Point p, geometry_msgs::Point q, geometry_msgs::Point r)
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
double orientation(geometry_msgs::Point p, geometry_msgs::Point q, geometry_msgs::Point r)
{
   double val = static_cast<double>(
                (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y)
                );

    if (val == 0) return 0;  // colinear

    return (val > 0)? 1: 2; // clock or counterclock wise
}

// The main function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool doIntersect(geometry_msgs::Point p1, geometry_msgs::Point q1, geometry_msgs::Point p2, geometry_msgs::Point q2)
{
    // Find the four orientations needed for general and
    // special cases
   double o1 = orientation(p1, q1, p2);
   double o2 = orientation(p1, q1, q2);
   double o3 = orientation(p2, q2, p1);
   double o4 = orientation(p2, q2, q1);

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