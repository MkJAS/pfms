#ifndef SHAPE_H
#define SHAPE_H

#include <string>
#include "line.h"

#include "geometry_msgs.h"

//http://wiki.ros.org/CppStyleGuide#Enumerations
namespace shape {

    typedef enum {
      CIRCLE=0,
      POLYGON=1
    } Type; /*!< Available shape categories*/

    const double MAX_AREA = 10.0;/*!< Default max area [m2] of cell*/

}
/*!
 *  \ingroup   ac_shape Shape
 *  \brief     Shape base class
 *  \details
 *  This class is the base class for all shapes.\n
 */
class Shape
{
public:
    Shape(geometry_msgs::Point position, unsigned int sides);

    virtual ~Shape();//Given this is an abstract class we should make a virtiual destructor

    /**
     * @brief Set the centre of the shape
     * @param position contains x and y position in [m]
     */
    void setCentre(geometry_msgs::Point position);

    /**
     * @brief Gets the centre of the shape
     * @return position of centre x and y in [m]
     */
    geometry_msgs::Point getCentre();

    /**
     * @brief Function that returns the type of shape
     * @return type of shape
     */
    shape::Type getType();

    /**
     * @brief Returns the decsrption of the  shape
     * @return string which describes the shape
     */
    std::string getDescription();
    /**
     * @brief Function that returns area
     * @return area in [m2]
     */
    virtual double getArea() = 0;
    /**
     * @brief Returns width and height at longest section of shape
     * @param width along x in [m]
     * @param height along y in [m]
     */
    virtual void getWidthHeight(double& width, double& height) =0;
    /**
     * @brief Returns number of sides (sides have constrant curvature or stright lines)
     * @return number of sides
     */
    virtual unsigned int getSides();
protected:
    double area_; // The area
    geometry_msgs::Point position_;//!< X,Y coordinate of centre of shape
    shape::Type type_;//!< type of shape, as per enum Type
    unsigned int sides_;//!< type of continous line sides for shape (if the line changes direction or cuvature its a new segment
    std::string description_;//!< description of shape
};

#endif // SHAPE_H
