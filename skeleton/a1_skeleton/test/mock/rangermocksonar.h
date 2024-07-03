#ifndef RANGERMOCKSONAR_H
#define RANGERMOCKSONAR_H

#include "rangerinterface.h"
#include "sonar.h"
#include <vector>

/*!
 *  \ingroup   ac_mock Mock Classes
 *  \brief     Mock Sonar Class
 *  \details
 *  This class is used to inject readings, and override the data of Sonar Class.\n
 *  It inherits from Sonar, however can store mock data to test Fusion Class.\n
 *  \author    Alen Alempijevic
 *  \author    Alex Virgona
 *  \version   1.01-2
 *  \date      2019-07-10
 *  \pre       none
 *  \bug       none reported as of 2020-04-11
 *  \warning
 */


class RangerMockSonar: public Sonar
{
public:
  RangerMockSonar();
  //First three are for ares, offset, fov - last is for mock data
  RangerMockSonar(unsigned int fov, unsigned int res, ranger::SensorPose sensor_pose, std::vector<double>);

  std::vector<double> generateData();

protected:
  std::vector<double> mockData_;
};

#endif // RANGERMOCKSONAR_H
