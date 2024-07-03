/**
 * @file    geometry.h
 * @ingroup Helper
 * @class   Geometry
 * @brief   Performs geometrical 2D calculations
 * @author  Ajal Singh
 * @version 1.0
 * @date    June 2020
 */

#ifndef GEOMETRY_H
#define GEOMETRY_H

//_____________________________________________________________________________ Includes
#include <iostream>
#include <cmath>

//_____________________________________________________________________________ Class Definition
class Geometry{
//_____________________________________________________________________________ Class Public Members

public:
    /**
     * @brief Construct a new Geometry object
     * 
     */
    Geometry();

    /**
     * @brief Destroy the Geometry object
     * 
     */
    ~Geometry();

    /**
     * @brief Get the Dist To Goal object
     * 
     * @param p1_x Point 1 x coordiante
     * @param p1_y Point 1 y coordinate
     * @param p2_x Point 2 x coordinate
     * @param p2_y Point 2 y coordinate
     * @return std::pair<double, double> {x distance, y distance}
     */
    std::pair<double, double> getDistToGoal(const double p1_x, const double p1_y, const double p2_x, const double p2_y );

    /**
     * @brief Get the Angle To Goal object
     * 
     * @param dist_x x distance from start to goal
     * @param dist_y y distance from start to goal
     * @param start_yaw starting yaw position
     * @return angle difference to goal point
     */
    double getAngleToGoal(const double dist_x, const double dist_y, const double start_yaw);

    /**
     * @brief Get the Angle Diff Goal Yaw object
     * 
     * @param goal_yaw 
     * @param start_yaw 
     * @return double angle difference to goal orientation
     */
    double getAngleDiffGoalYaw(const double goal_yaw, const double start_yaw);

//_____________________________________________________________________________ Class Private Members
private:
//_____________________________________________________________________________ Private Member Functions
    /**
     * @brief normalise angles between -Pi to Pi
     * 
     * @param angle 
     * @return normalised angle 
     */
    double normaliseAngle(double angle);    
};
#endif //GEOMETRY_H