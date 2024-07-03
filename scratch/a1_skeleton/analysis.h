#ifndef ANALYSIS_H
#define ANALYSIS_H
#include <vector>


/**
 * @file
 * @brief The functions implemented to perform grabandfuse data function from rangerfusion
 * along with caclulating the sonar union
 * 
 */

/*!
 *  
    \brief     Analysis header
 *  \details
 *  The functions implemented to perform grabandfuse data function from rangerfusion
 */

namespace geometry_msgs {

    struct Point{
        double x;
        double y;
    };/*!< Structure for 2d Positions*/
}

/**
 * @brief Finds the intersection points of two lines given by the points AB and CD
 * 
 * @param A 
 * @param B 
 * @param C 
 * @param D 
 * @return geometry_msgs::Point intersection points
 */
geometry_msgs::Point PointofLineIntersection(geometry_msgs::Point A,geometry_msgs::Point B,geometry_msgs::Point C, 
geometry_msgs::Point D);

/**
 * @brief           Checks if any side of sonar i intersects with any sides of sonar i+1 by 
 *                  calling PointofLineIntersection for each pair of sides (9 pairs)
 * @param a         3 sides of sonar i
 * @param b 
 * @param c 
 * @param a1        3 sides of sonar i+1
 * @param b1 
 * @param c1 
 * @param all_POI   vector containing all points of intersection
 * @return true     returns true to indicate that the sonar does intersect with another sonar
 * @return false    returns false to indicate that the sonar does NOT intersect with another sonar
 */
bool Checkallsides(geometry_msgs::Point a,geometry_msgs::Point b,geometry_msgs::Point c,geometry_msgs::Point a1,
geometry_msgs::Point b1,geometry_msgs::Point c1,std::vector<geometry_msgs::Point> &all_POI);

/**
 * @brief Checks if any of the intersect points are inside the bounds of a sonar using 
 * Barycentric coordinate calculations
 * 
 * @param sonarStart 
 * @param sonarCorner1 
 * @param sonarCorner2 
 * @param a 
 * @return true 
 * @return false 
 */
bool isInsideSonar(geometry_msgs::Point sonarStart,geometry_msgs::Point sonarCorner1,
geometry_msgs::Point sonarCorner2,geometry_msgs::Point a);

/**
 * @brief Using the functions below, function determines if the line between the two laser points intersects with
 * any of the cell edges defines by ab,bc,cd,da
 * 
 * @param laserStart 
 * @param laserEnd 
 * @param a 
 * @param b 
 * @param c 
 * @param d 
 * @return true 
 * @return false 
 */

bool LaserIntersect(geometry_msgs::Point laserStart, geometry_msgs::Point laserEnd,geometry_msgs::Point a,
geometry_msgs::Point b,geometry_msgs::Point c,geometry_msgs::Point d);

/**
 * @brief Establish a logic that if the sonar is to state a cell is OCCUPIED, that the curve of the sonar, or
 * the end of the cone MUST then exist within the bounds of the cell. In this case, we implement a function
 * in which it obtains points along the curve and checks if any of these points lie within a set margin
 * around the cells edges
 * 
 * @param sonarStart 
 * @param sonarCorner 
 * @param a top left corner
 * @param b bottom left corner
 * @param c bottom right corner
 * @param d top right corner
 * @return true 
 * @return false 
 */
bool SonarOccupied(geometry_msgs::Point sonarStart, geometry_msgs::Point sonarCorner,double theta,double sonarangle,
double radius, geometry_msgs::Point a, geometry_msgs::Point b,geometry_msgs::Point c,geometry_msgs::Point d);


/**
 * @brief Essentially, by converting the cell corner points into polar coordinates, a cell corner exists within the
 * bounds of a sonar if the corner is a distance from the sonar start that is less than the sonar reading (radius)
 * AND the corner point is also within the 2 angle ranges from the positive x axis set by the 2 sonar edges
 * E.g a sonar with 20 deg FOV facing forward along y axis would set angle bounds between 100 and 80 degrees
 * If a cell corner exists WITHIN the sonar bounds BUT is NOT found to be occupied it must then be FREE
 * 
 * @param theta 
 * @param radius 
 * @param sonarangle 
 * @param sonarStart 
 * @param sonarCorner 
 * @param a 
 * @param b 
 * @return true 
 * @return false 
 */
bool SonarFree(double theta, double radius,double sonarangle,geometry_msgs::Point sonarStart,
geometry_msgs::Point sonarCorner, geometry_msgs::Point a, geometry_msgs::Point b);

//
/**
 * @brief Given three colinear points p, q, r, the function checks if
 *  point q lies on line segment 'pr'
 * @param p colinear point
 * @param q colinear point
 * @param r colinear point
 * @return boolean indicating point q lies on line segment 'pr'
 * @note atttribution to https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
 */
bool onSegment(geometry_msgs::Point p, geometry_msgs::Point q, geometry_msgs::Point r);

/**
 * @brief To find orientation of ordered triplet (p, q, r).
 * @param p point
 * @param q  point
 * @param r  point
 * @return boolean
 * 0 --> p, q and r are colinear
 * 1 --> Clockwise
 * 2 --> Counterclockwise
 * @note atttribution to https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
 */
double orientation(geometry_msgs::Point p, geometry_msgs::Point q, geometry_msgs::Point r);


/**
 * @brief Function that returns true if line segment 'p1q1' intersects 'p2q2'
 * @param p1 first point of line 1
 * @param q1 second point of line 1
 * @param p2 first point of line 2
 * @param q2 second point of line 2
 * @return boolean indicating lines intersect
 * @note atttribution to https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
 */
bool doIntersect(geometry_msgs::Point p1, geometry_msgs::Point q1, geometry_msgs::Point p2, geometry_msgs::Point q2);


#endif // ANALYSIS_H