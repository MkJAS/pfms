#ifndef LASERPROCESSING_H
#define LASERPROCESSING_H

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include "tf/transform_datatypes.h"

class LaserProcessing
{
public:
  /*! @brief Constructor that allocates internals
   *
   *  @param[in]    laserScan - laserScan to be processed
   */
  LaserProcessing(sensor_msgs::LaserScan laserScan);

  /*! TASK1
   * @brief Count number of high intensity reading
   *
   * @return the number of high intensity readings in current laser scan
   */
  unsigned int countHighIntensity();

  /*! TASK 2
   *  @brief Count number of high intensity segments
   *  @note Segments are formed by high intensity readings.
   * A segment is a sequence (consecutive) high intensity readings that are less than 0.3m
   * away from each other, refer image in README.md
   *
   * @return the number of segments in the current laser scan
   */
  unsigned int countSegments();

  /*! TASK 3
   * @brief Return position of first high intensity reading
   *
   * The position should be the location of the first high intensity readins
   * @return point - point at location of first high intensity reading
   */
  geometry_msgs::Point detectPositionHighIntensity();


  /*! TASK 4
   * @brief Return a pose at first high intensity readings
   *
   * In the pose, the position should be on the first readings
   * While the orientation should be
   * looking towards the next 3 readigs
   * @return pose - pose at the location of firts high intensity reading, orinetated towards others
   */
  geometry_msgs::Pose detectPoseHighIntensity();


  /*! @brief Accepts a new laserScan
   *  @param[in]    laserScan  - laserScan to be processed
   */
  void newScan(sensor_msgs::LaserScan laserScan);

private:
  /*! @brief Returns the cartesian position of laer reading at specific index
   * converted from polar coordinats stored in the #laserScan_
   *  @param[in] index - the reading needing conversion
   *  @return position cartesian values
   */
   geometry_msgs::Point polarToCart(unsigned int index);

   /*! @brief Given two points (only x,y are used), returns the slope slope of the lines connecting them
    *  @param[in] p1 - first point
    *  @param[in] p2 - second point
    *  @return slope of line between the points in radians
    */
  double angleConnectingPoints(geometry_msgs::Point p1, geometry_msgs::Point p2);

private:
  sensor_msgs::LaserScan laserScan_;

};

#endif // DETECTCABINET_H
