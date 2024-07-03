#include "rangermocklaser.h"

RangerMockLaser::RangerMockLaser() {
}

RangerMockLaser::RangerMockLaser(unsigned int fov, unsigned int res, ranger::SensorPose sensor_pose, std::vector<double> mockData) {
  //Extra constructor for easy mocking
  mockData_ = mockData;

  Laser::setAngularResolution(res);
  Laser::setSensorPose(sensor_pose);
  Laser::setFieldOfView(fov);
}


std::vector<double> RangerMockLaser::generateData() {
  return mockData_;
}
