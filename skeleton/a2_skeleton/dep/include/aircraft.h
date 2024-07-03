/*! @file
 *
 *  @brief Aircraft specific structure used for simulator.
 *  Students could, but not necessarily need to use this header and structures.
 *
 *  @author alalemp
 *  @date 15-09-2021
*/
#ifndef AIRCRAFT_H
#define AIRCRAFT_H

#include <vector>
#include <mutex>
#include <cmath>
#include "timer.h"
#include "types.h"

using geometry_msgs::Pose;
using geometry_msgs::Point;

namespace aircraft {

    /* Enum for the Aicraft state machine */
    enum AircraftState {
      BS_START=0,
      BS_STRAIGHT=1,
      BS_TURNING=2,
      BS_RECOVERY=3,
      BS_UNKNOWN=-1,
    };


    struct Aircraft {
      Pose pose;                    /*!< Global position and orientation within the airspace */
      std::vector<Point> trail; /*!< To display where the aircraft has been */
      double linear_velocity;       /*!< Linear velocity (metres/second) */
      double angular_velocity;      /*!< Angular velocity (radians/second). (+) Counter clockwise, (-) Clockwise. */
      Timer timer;                  /*!< Used to keep track of elapsed time. */
      AircraftState state;          /*!< Aircraft state during operation. */
      Pose currentGoalPose;         /*!< Current global goal position and orientation within the airspace. */
      Pose previousGoalPose;        /*!< Previous global goal position and orientation within the airspace. */
    };

    struct AircraftContainer { /*!< A thread safe container for the Aircraft type */
      std::vector<Aircraft> a;
      std::mutex access;
    };
}
#endif // AIRCRAFT_H
