/**
 * @file    purePursuit.h
 * @ingroup Control
 * @class   PurePursuit
 * @brief   Performs pure pursuit between two points and returns gamma (arc).
 * @author  Ajal Singh
 * @version 1.0
 * @date    June 2020
 */

#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

#include "geometry_msgs/PoseArray.h"

//_____________________________________________________________________________ Class Definition
class PurePursuit{
//_____________________________________________________________________________ Class Public Members
public:
    /**
     * @brief Construct a new Pure Pursuit object
     * 
     */
    PurePursuit();

    /**
     * @brief Destroy the Pure Pursuit object
     * 
     */
    ~PurePursuit();
    
    /**
     * @brief Performs pure pursuit to calcualte gamma (arc)
     * 
     * @param current_pose the current Pose or start position
     * @param goal_pose the goal position
     */
    void doPurePursuit(const geometry_msgs::Pose & current_pose, const geometry_msgs::Pose &goal_pose);
    
    /**
     * @brief Get the Gamma object
     * 
     * @return double gamma
     */
    double getGamma();

    //_____________________________________________________________________________ Class Private Members
private:
    double gamma_;  //!< arc or curve
};
#endif //PUREPURSUIT_H