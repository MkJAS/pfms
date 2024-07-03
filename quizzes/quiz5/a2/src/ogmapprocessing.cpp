#include "ogmapprocessing.h"
#include <ros/console.h>

OgmapProcessing::OgmapProcessing(nav_msgs::OccupancyGrid map) :
  map_(map)
{

}

//! @todo
//! TASK 1 - Refer to README.md and the Header file for full description
unsigned int OgmapProcessing::countUnknwonCells(){

  unsigned int count = 0;
  //std::vector<int> map = map_.data();

  for(auto it:map_.data)
  {
    if(it==-1)
    {
      count++;
    }
  }



  return count;

}

//! @todo
//! TASK 2 - Refer to README.md and the Header file for full description
bool OgmapProcessing::isLocationFree(geometry_msgs::Point goal){

  bool free =false;
  geometry_msgs::Point point =  map_.info.origin.position;


  int col = static_cast<int>((goal.x - point.x)/map_.info.resolution);
  int row = static_cast<int>((goal.y - point.y)/map_.info.resolution);
  int index = row * map_.info.width + col;
  //ROS_INFO_STREAM("index "<<static_cast<int>(map_.data.at(index)));
  if(static_cast<int>(map_.data.at(index))==0)
  {
    free = true;
  }


  return free;

}

void OgmapProcessing::newMap(nav_msgs::OccupancyGrid map){
  map_=map;
}
