#ifndef ANALYSIS_H
#define ANALYSIS_H

#include "analysisinterface.h"

class Analysis : public AnalysisInterface
{
public:
  Analysis(std::vector<Shape*> shapes,Line line);

  //Returns a container of bools that indicates if the correspoing shape intersects the line
  /**
   * @brief Checks wether the shapes_ and line_ intersect
   * @param vector of booleans, same size and the number of shapes_
   */
  std::vector<bool> intersectsLine();

  /**
   * @brief Return a vector with the count of circles, triangles and vectors respectively
   * @return the vector should have 3 elements, element 0 for circles, 2 for triangles and 3 for rectangles
   */
  std::vector<int> detectShapes();


private:
  /**
   * @brief Checks the intercept between circle and line.
   * THIS FUNCTION DOES NOT FORM ANY OF THE QUIZ ASSESSMENTS - Here for fun!
   * @return boolean indicating if there is an intercept
   */
    bool intersectsCircle(Shape* shape);


    /**
     * @brief Checks the intercept between rectangle and line. It uses the corners of the rectangle
     * and checks for each of the four sides that connect the corners wether they intercept the line_
     * @return boolean indicating if there is an intercept
    */
    bool intersectsRectangle(Shape* shape);
    /**
     * @brief Checks the intercept between triangle and line. It uses the corners of the triangle
     * (given we have the width and height and it's an isosceles triangle) therefore it
     * checks for each of the three sides that connect the corners wether they intercept the line_
     * @return boolean indicating if there is an intercept
     * @note atttribution to https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
     */
    bool intersectsTriangle(Shape* shape);


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
    int orientation(geometry_msgs::Point p, geometry_msgs::Point q, geometry_msgs::Point r);

    //
    /**
     * @brief function that returns true if line segment 'p1q1' intersects 'p2q2'
     * @param p1 first point of line 1
     * @param q1 second point of line 1
     * @param p2 first point of line 2
     * @param q2 second point of line 2
     * @return boolean indicating lines intersect
     * @note atttribution to https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
     */
    bool doIntersect(geometry_msgs::Point p1, geometry_msgs::Point q1, geometry_msgs::Point p2, geometry_msgs::Point q2);

private:
  std::vector<Shape*> shapes_;
  Line line_;

};

#endif // ANALYSIS_H
