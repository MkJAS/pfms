#ifndef STRUCTS_H
#define STRUCTS_H

#include <string>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h" 

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "project_setup/RequestGoal.h"

struct FnFData
{
    geometry_msgs::Pose pose0;
    geometry_msgs::Pose pose1;
    sensor_msgs::LaserScan laserscan;
};

#endif