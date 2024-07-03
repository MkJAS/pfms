#ifndef RANGERMOCKLASER_H
#define RANGERMOCKLASER_H

#include "rangerinterface.h"
#include "laser.h"
#include <vector>

/*!
 *  \ingroup   ac_mock Mock Classes
 *  \brief     Mock Sonar Class
 *  \details
 *  This class is used to inject readings, and override the data of Laser Class.\n
 *  It inherits from Laser, however can store mock data to test Fusion Class.\n
 *  \author    Alen Alempijevic
 *  \author    Alex Virgona
 *  \version   1.01-2
 *  \date      2019-07-10
 *  \pre       none
 *  \bug       none reported as of 2020-04-11
 *  \warning
 */

class RangerMockLaser: public Laser
{
public:
  RangerMockLaser();
  //First three are for ares, offset, fov - last is for mock data
  RangerMockLaser(unsigned int fov, unsigned int res, ranger::SensorPose sensor_pose, std::vector<double> mockData);

  std::vector<double> generateData();

protected:
  std::vector<double> mockData_;
};

#endif // RANGERMOCKLASER_H
