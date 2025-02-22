#ifndef SHAPE_H
#define SHAPE_H

#include <string>

/*!
 *  \ingroup   ac_shapre Shape
 *  \brief     Shape base class
 *  \details
 *  This class is the base class for all shapes.\n
 */
class Shape
{

public:
    Shape();
    void setCentre(double x, double y);
    void offSetCentre(double x, double y);
    std::string getDescription();
    //void getCentre(double& x, double& y);

    virtual bool checkIntercept(double x, double y) = 0;

protected:
    std::string description_;//!< description of shape


//private:
    double centreX_;//!< X coordinate of centre of shape
    double centreY_;//!< Y coordinate of centre of shape
};

#endif // SHAPE_H
