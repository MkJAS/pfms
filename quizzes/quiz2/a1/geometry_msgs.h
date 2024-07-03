#ifndef GEOMETRY_MSGS_H
#define GEOMETRY_MSGS_H

namespace geometry_msgs {

    struct Point{
        double x;
        double y;
    };/*!< Structure for 2d Positions*/

    struct Pose2D{
        double x;
        double y;
        double theta;
    };/*!< Structure for 2d Pose*/

}

#endif // GEOMETRY_MSGS_H
