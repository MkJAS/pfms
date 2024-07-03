#ifndef OGMAPPROCESSING_H
#define OGMAPPROCESSING_H

//ROS Message Types
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>

class OgmapProcessing
{
public:
  /*! @brief Constructor that allocates internals
   *
   *  @param[in]    map - OccupancyGrid map to be processed
   */
  OgmapProcessing(nav_msgs::OccupancyGrid map);

  /*! TASK1
   * @brief Count number of uknown cells
   *
   * @return the number of uknwon cells in current map
   */
  unsigned int countUnknwonCells();

  /*! TASK 2
   *  @brief Detect if position at this location in OgMap is free
   * @param global position (point) that is examined
   * @return if the cell is free
   */
  bool isLocationFree(geometry_msgs::Point goal);

  /*! @brief Accepts a new map
   *  @param[in]    map  - OccupancyGrid to be processed
   */
  void newMap(nav_msgs::OccupancyGrid map);




private:
 nav_msgs::OccupancyGrid map_;

};

#endif // OGMAPPROCESSING_H
