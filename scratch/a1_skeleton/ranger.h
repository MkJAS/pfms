#ifndef RANGER_H
#define RANGER_H

#include <string>
#include "rangerinterface.h"
#include <iostream>
#include <math.h>

class Ranger: public RangerInterface
{
public:
  //Default constructors should set all sensor attributes to a default value
  Ranger();

  std::vector<double> generateData();

  unsigned int getAngularResolution(void);

 
  bool setAngularResolution(unsigned int resolution);

  ranger::SensorPose getSensorPose(void);


  bool setSensorPose(ranger::SensorPose pose);

  unsigned int getFieldOfView(void);


 bool setFieldOfView(unsigned int fov);

 double getMaxRange(void);

 double getMinRange(void);
 
ranger::SensingMethod getSensingMethod(void);
/**
 * @brief Prints out the parameters of the sensor that cannot be altered
 * 
 */
 virtual void getSetParams(void);

 
/**
 * @brief Get the Num Samples object or Sequence Number
 * 
 * @return double 
 */
double getSequenceNumber(void);


protected:
  double maxrange_;
  double minrange_;
  unsigned int fov_;
  ranger::SensorPose pose_;
  unsigned int resolution_;
  ranger::SensingMethod method_;
  std::string modelname_;
  double numReadings_; //number of readings a sensor should have. Used for generating data
  double numSamples_;
  bool fov_set_;
};

#endif // RANGER_H
