#ifndef FINDNFOLLOWADV_H
#define FINDNFOLLOWADV_H

#include "findnfollow_int.h"

/*!
 *  \brief     Find n Follow advanced mode class
 *  \details   Contains the overloaded start and follow functions from the FnF interface class as well as the
 *              additional getpointbehindR1 and standbeside functions
 *  \author    Joseph Seklawy
 */

class FnFadv : public FnFint
{
public:
    /**
     * @brief Construct a new FnFadv object, instatiates the class booleans and calls the start function
     * 
     */
    FnFadv();

    FnFadv(bool threads);
    /**
     * @brief Destroy the FnFadv object
     * 
     */
    ~FnFadv();
    /**
     * @brief Obtains the points to the left and right of robot_1 and determines whether robot_0 can reach one of them
     * If robot_0 can, it moves to it. If it cannot reach either side it should simply get as close to robot_1 
     * 
     */
    geometry_msgs::PoseArray standbeside();

private:
    /**
     * @brief Starts the required threads for advanced mode
     * @sa search
     * @sa following
     * @sa getRobot1poses
     * @sa distancecheck
     */
    void start();

    /**
     * @brief Follows robot_1 by trying to stay directly behind it
     * 
     */
    void following(); 
    
    /**
     * @brief A thread which checks the pose of robot_1 every second. If robot_1 has remaind in the same position for more
     * than 10 seconds, it will launch the standbeside function 
     * @sa standbeside
     */
    void getRobot1poses();

    /**
     * @brief A thread which checks the distance between robot_1 and robot_0 to ensure robot_0 doesnt collide into 
     * robot_1. If robot_1 is too close, robot_0 will remain still
     * 
     */
    void distancecheck();

    std::atomic<bool> beside_;
    std::atomic<bool> behind_;
    
};



#endif