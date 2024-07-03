#include "tf2.h"
#include "tf.h"
#include <cmath> // for trig operations

namespace tf2 {

    //! @todo
    //! TASK 1 - Refer to README.md and the Header file for full description
    Point local2Global(RangeBearingStamped rangeBearing, Pose aircraft)
    {
      Point p;
      Point local;
      local.x = aircraft.position.x;
      local.y = aircraft.position.y;
      double theta = tf::quaternionToYaw(aircraft.orientation);

      Point Temp;
      Temp.x = rangeBearing.range*cos(2*M_PI-rangeBearing.bearing);
      Temp.y = rangeBearing.range*sin(M_PI-rangeBearing.bearing);

      p.x = local.x + Temp.x*cos(theta)-Temp.y*sin(theta);
      p.y = local.y + Temp.x*sin(theta)+Temp.y*cos(theta);
  
      return p;

    }

    //! @todo
    //! TASK 2 - Refer to README.md and the Header file for full description
    RangeBearingStamped global2local(Point bogie, Pose aircraft)
    {
      RangeBearingStamped rbstamped = {0, 0,0};
      double theta = tf::quaternionToYaw(aircraft.orientation);
      
      double x = (bogie.x-aircraft.position.x)*cos(theta)+(bogie.y-aircraft.position.x)*sin(theta);
      double y = -1*(bogie.x-aircraft.position.x)*sin(theta)+(bogie.y-aircraft.position.x)*cos(theta);

      rbstamped.range = pow((pow(x,2)+pow(y,2)),0.5);
      rbstamped.bearing = 2*M_PI - acos(x/rbstamped.range);

      return rbstamped;
    }


    double normaliseAngle(double theta) {
      if (theta > (2 * M_PI))
        theta = theta - (2 * M_PI);
      else if (theta < 0)
        theta = theta + (2 * M_PI);

      return theta;
    }

}

