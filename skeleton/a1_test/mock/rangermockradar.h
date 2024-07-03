#ifndef RANGERMOCKSONAR_H
#define RANGERMOCKSONAR_H

#include "../../rangerinterface.h"
#include "../../sonar.h"
#include <vector>

class RangerMockSonar: public Sonar
{
public:
  RangerMockSonar();
  //First three are for ares, offset, fov - last is for mock data
  RangerMockSonar(unsigned int, unsigned int, int, std::vector<double>);

  unsigned int getAngularResolution(void);
  int getOffset(void);
  unsigned int getFieldOfView(void);

  bool setAngularResolution(unsigned int);
  bool setOffset(int);
  bool setFieldOfView(unsigned int);

  SensingMethod getSensingMethod(void);

  std::vector<double> generateData();

protected:
  unsigned int fieldOfView_;
  unsigned int angularResolution_;
  int offset_;

  std::vector<double> mockData_;
};

#endif // RANGERMOCKSONAR_H
