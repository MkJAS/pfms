#ifndef FINDNFOLLOW_H
#define FINDNFOLLOW_H


#include <sstream>
#include <iostream>
#include <string>
#include <vector>

#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h" 

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "project_setup/RequestGoal.h"
#include "visualization_msgs/MarkerArray.h"

#include "utilities.h"
#include "grid_processing.h"
#include "findnfollow_int.h"

/*!
 *  \brief     Find n Follow basic mode class
 *  \details   Contains the overloaded start and follow functions from the FnF interface class
 *  \author    Joseph Seklawy
 */


class FnF : public FnFint
{
public:
    /**
     * @brief Default Constructor a new FnF object and instatiates all class booleans
     * 
     */
    FnF();

    /**
     * @brief Construct a new Fn F object
     * 
     * @param start 
     */

    FnF(bool threads);
    /**
     * @brief Destroy the FnF object and end threads cleanly
     * 
     */
    ~FnF();

protected: 
    /**
     * @brief Starts the required threads for basic mode
     * @sa search
     * @sa following
     * 
     */
    void start();

    /**
     * @brief Once robot_1 has been found this function begins calculating the points around robot_1 for robot_0 to follow
     * @sa getpointaroundR1
     */
    void following(); 
};

#endif