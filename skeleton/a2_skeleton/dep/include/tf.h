/*! @file
 *
 *  @brief Collection of transforms between quaterion and yaw
 *
 *
 *  @author Alen Alempijevic
 *  @date 10-09-2021
*/
#ifndef TF_H
#define TF_H

#include "types.h"

//! The tf namespace
namespace tf {

    /*! @brief Returns yaw from a quaternion
     *
     *  Calculates quaternion from yaw (assumes there is no roll
     *  or picth (constrained to 2d).
     *
     *  @param yaw - in [rad]
     *  @return Quaternion
    */
    geometry_msgs::Quaternion yawToQuaternion(double yaw);

    /*! @brief Returns yaw from a quaternion
     *
     *  Calculates the yaw from quaternion (assumes there is no roll
     *  or picth (constrained to 2d).
     *
     *  @param q - Quaternion
     *  @return yaw - in [rad]
    */
    double quaternionToYaw(geometry_msgs::Quaternion q);
}

#endif // TF_H
