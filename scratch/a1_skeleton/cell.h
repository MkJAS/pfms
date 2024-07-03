#ifndef CELL_H
#define CELL_H

/*!
 *  \brief     Cell Class
 *  \details
 *  This class is used to describe cells that are used in fusion.\n
 *  \author    Alen Alempijevic
 *  \version   1.01-2
 *  \date      2019-07-10
 *  \pre       none
 *  \bug       none reported as of 2020-04-11
 *  \warning   students MUST NOT change this class (the header or implementation file)
 */



//http://wiki.ros.org/CppStyleGuide#Enumerations
namespace cell {

typedef enum {
  UNKNOWN=0,
  FREE=1,
  OCCUPIED=-1
} State; /*!< Available cell states*/

const double MAP_SIZE = 10.0;/*!< Default map size [m] to draw cells from*/
const double DEFAULT_CELL_SIZE = 0.1;/*!< Default cell size [m]*/

}

/**
@class Cell
@brief Cells will be used as areas of space (rectangles) where the sensor data will be fused to indicate occupancy, default size provided, centre location draw randonly from a map of max size. On creation state is UNKNOWN
@details The Cell will be used for fusion accordinly: \n
If the sensor intersects the cell and goes through Cell will be FREE\n
If the sensor has a return from within Cell, it will be OCCUPIED\n*/
class Cell {

// Public members are accessible from outside the class (ie. in main)
public:
  /**
  The Default constructor sets the cell centre to values within the #MAP_SIZE\n
  @note NOTE: The  cell size is also stated as #DEFAULT_CELL_SIZE
  */
    Cell();

    /**
    Member function sets Side
    @param side The desired side length
    */
    void setSide (double side);

    /**
    Member function to get value of the side of cell
    @return side of cell [m]
    */
    double getSide (void);

    /**
    Member function to get area of cell
    @return area of cell  [m2]
    */
    double area(void);

    /**
    Member function to get perimeter of cell
    @return perimieter of cell [m]
    */
    double perimeter(void);

    /**
    Member function to set centre of cell
    @param x centre coordinate x [m]
    @param y centre coordinate y [m]
    */
    void setCentre(double x, double y);
    /**
    Member function to get centre of cell
    @param x centre coordinate x [m]
    @param y centre coordinate y [m]
    */
    void getCentre(double &x, double &y);

    /**
    Member function to get state of cell
    @return state of cell
    */
    cell::State getState();

    /**
    Member function to set state of cell
    @param area of cell
    */
    void setState(cell::State);

// Private members are only accessible from within methods of the same class
private:
    double side_;   //!< Sides of the cell (which is rectangle shape so has equal width and height)
    double centreX_;//!< X coordinate of centre of shape
    double centreY_;//!< Y coordinate of centre of shape
    cell::State state_;   //!< State of cell

};

#endif // CELL_H
