/**
 * @file    controller_helper.h
 * @ingroup Helper
 * @class   VelocityControlHelper
 * @brief   Assists velocity control class 
 * @author  Ajal Singh
 * @version 1.0
 * @date    June 2020
 */

#ifndef CONTROLLER_HELPER_H
#define CONTROLLER_HELPER_H

//_____________________________________________________________________________ Includes
#include "sensor_msgs/LaserScan.h"

//_____________________________________________________________________________ Class Definition
class VelocityControlHelper{

//_____________________________________________________________________________ Class Public Members
public:
    /**
     * @brief Construct a new Velocity Control Helper object
     * 
     */
    VelocityControlHelper();

    /**
     * @brief Destroy the Velocity Control Helper object
     * 
     */
    ~VelocityControlHelper();

    /**
     * @brief Uses laser scan data to determine whether vehicle has encountered an obstacle
     * 
     * @param msg 
     * @return true 
     * @return false 
     */
    bool checkForObstacle(const sensor_msgs::LaserScanPtr& msg);

};
#endif //CONTROLLER_HELPER_H