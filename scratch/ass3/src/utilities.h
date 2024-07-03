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
#include "grid_processing.h"


/**
 * @file
 * @brief Contains all the helper functions which are used by both the basic and advanced mode classes
 * 
 */


struct FnFData
{
    geometry_msgs::Pose pose0;
    geometry_msgs::Pose pose1;
    sensor_msgs::LaserScan laserscan;
};
//! Struct created simply to make passing data to functions a little neater
struct Pathpacket
{
    geometry_msgs::Pose current0;
    geometry_msgs::Pose current1;
    geometry_msgs::Pose point_around1;
    
};
//! Struct used to hold all info regarding robot_0s corners at the current time,position and at a given goal pose
struct Cornerpacket
{
    geometry_msgs::Point corner1;
    geometry_msgs::Point corner2;
    geometry_msgs::Point corner1c;
    geometry_msgs::Point corner2c;
    geometry_msgs::Point corner1g;
    geometry_msgs::Point corner2g;

};
/**
 * @brief takes an angle from the global x axis and converts it to a quaternion for use in Pose messages
 * 
 * @param yaw 
 * @param pose the pose requiring the quaternion
 */
void YawtoQuat(double yaw, geometry_msgs::Pose& pose);

/**
 * @brief Used in basic mode. Sets a circle of radius 1 around robot_1 and determines the closest point to robot_0
 * out of 16 equally spaced points along the circle
 * 
 * @param r0 current robot_0 pose
 * @param r1 current robot_1 pose
 * @return geometry_msgs::Point 
 */
geometry_msgs::Point getpointaroundR1(geometry_msgs::Point r0, geometry_msgs::Point r1);

/**
 * @brief Used in advanced mode. Determines the point 0.7 directly behind robot_1
 * 
 * @param r1 robot_1 pose
 * @return geometry_msgs::Point 
 */
geometry_msgs::Point getpointbehindR1(geometry_msgs::Pose r1);

/**
 * @brief converts a point in a given local frame to global coordinates
 * 
 * @param current local frame of reference
 * @param point   point in local frame (x,y)
 * @return geometry_msgs::Point converted point
 */
geometry_msgs::Point localtoglobal(geometry_msgs::Pose current, geometry_msgs::Point point);

/**
 * @brief Takes the corners of robot_0 and determines whether they can reach a given point unhindered.
 * This is done to determine whether the robot can do the same, as the robots bounds as it moves are its corners
 * 
 * @param current robot_0 current pose
 * @param goal    the goal pose
 * @param corner1 
 * @param corner2 
 * @param grid    an object of the GridProcessing class @ref grid_processing.h
 * @param reachable1 
 * @param reachable2 
 */
void checkCorners(geometry_msgs::Pose current, geometry_msgs::Pose goal, geometry_msgs::Point corner1, geometry_msgs::Point corner2, GridProcessing& grid,bool& reachable1,bool& reachable2);

/**
 * @brief Determines what type of corridor robot_1 may be in by checking a series of points along the x or y axis within the 
 * occupancy grid map. For eg, in the instance of a horizontal corridor, points along the y axis will be checked and if enough are able to reach
 * robot_1 unhindered, the function declares robot_1 to be in a horizontal corridor
 * 
 * \image html doc/pic/corridor.png width=25%
 * 
 * @param zero 
 * @param temp_goal 
 * @param check_type 
 * @param grid 
 * @return true 
 * @return false 
 */
bool Corridortype(geometry_msgs::Point zero, geometry_msgs::Point temp_goal,int check_type,GridProcessing& grid);


#endif