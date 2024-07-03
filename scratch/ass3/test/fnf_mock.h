#ifndef FNFMOCK_H
#define FNFMOCK_H


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
 *  @brief     Find n Follow mock class for utests
 *  @details   Contains the functions from both FnF classes to be tested. All functions
 *              listed here are exact copies of those found in both the FnF and FnFadv classes
 *              and any tests done here are as such indicitive of the performance of functions in the actual classes
 *  @author    Joseph Seklawy
 */


class FnF_mock
{
public:
    FnF_mock();

    void setData(FnFData data, nav_msgs::OccupancyGrid grid);
    bool IsinLoS();
    geometry_msgs::PoseArray getpath(Pathpacket pack);
    geometry_msgs::PoseArray standbeside();
    geometry_msgs::PoseArray goaround(Pathpacket pack,geometry_msgs::Point corner1,geometry_msgs::Point corner2);


private:
    FnFData data_;
    nav_msgs::OccupancyGrid Grid_;
    geometry_msgs::PoseArray R1poses_;
    //! Bool set when robot_1 is first in sight    
    std::atomic<bool> insight_;
    std::atomic<bool> moving_;
    bool running;
    std::condition_variable cv_;
    std::condition_variable Followcv_;
    std::atomic<bool> datagot_;
    std::atomic<bool> following_;
    std::atomic<bool> searching_;
    std::atomic<bool> interrupt_;
    std::atomic<bool> lost_;
    std::atomic<bool> beside_;
    std::atomic<bool> behind_;

    std::mutex mtx_;
    std::mutex Followmtx_;
    std::mutex R1mtx_;
    //!diameter of robot_0
    double diameter; 

};


#endif