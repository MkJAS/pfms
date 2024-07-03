#include "rangermocklaser.h"
#include <cmath>

RangerMockLaser::RangerMockLaser() {
}

RangerMockLaser::RangerMockLaser(unsigned int fov, unsigned int res, ranger::SensorPose pose, std::vector<double> mockData) {
  //Extra constructor for easy mocking
  mockData_ = mockData;

  Laser::setAngularResolution(res);
  Laser::setSensorPose(pose);
  Laser::setFieldOfView(fov);
}

RangerMockLaser::RangerMockLaser(unsigned int fov, unsigned int res, int offset, std::vector<double> mockData) {
  //Extra constructor for easy mocking
  mockData_ = mockData;

  Laser::setAngularResolution(res);
  ranger::SensorPose pose{0,0,offset*M_PI/180};
  Laser::setSensorPose(pose);
  Laser::setFieldOfView(fov);
}


std::vector<double> RangerMockLaser::generateData() {
  return mockData_;
}
