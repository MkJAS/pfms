#ifndef CIRCLE_H
#define CIRCLE_H
#include <vector>


class Circle
{
public:
  Circle(double radius);
  void setradius (double radius);

  double getarea (void);
  void setarea (double radius);

  double getperimeter (void);
  void setperimeter (double radius);

private:
  double radius_;
  double area_;
  double perimeter_;

};

// double CirclesArea(std::vector<Circle> CircVec);
// /**
//  * @brief Function that retruns a vector containing elements of an initial vector that are less than a value
//  * @param CircVec vector to be searched
//  * @param total_area area of all circles
//  * @return area of all circles
//  */

#endif // CIRCLE_H
