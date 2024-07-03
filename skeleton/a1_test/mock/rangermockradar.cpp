#include "rangermockradar.h"

RangerMockSonar::RangerMockSonar() {
}

RangerMockSonar::RangerMockSonar(unsigned int fov, unsigned int ares, int offset, std::vector<double> mockData) {
  //Extra constructor for easy mocking
  angularResolution_ = ares;
  offset_ = offset;
  fieldOfView_ = fov;
  mockData_ = mockData;

  Sonar::setAngularResolution(ares);
  Sonar::setOffset(offset);
  Sonar::setFieldOfView(fov);
}

unsigned int RangerMockSonar::getAngularResolution(void) {
  return angularResolution_;
}
int RangerMockSonar::getOffset(void) {
  return offset_;
}

unsigned int RangerMockSonar::getFieldOfView(void) {
  return fieldOfView_;
}

std::vector<double> RangerMockSonar::generateData() {
  return mockData_;
}

bool RangerMockSonar::setAngularResolution(unsigned int ares) {
  angularResolution_ = ares;
  return true;
}

bool RangerMockSonar::setOffset(int offset) {
  offset_ = offset;
  return true;
}

bool RangerMockSonar::setFieldOfView(unsigned int fov) {
  fieldOfView_ = fov;
  return true;
}

SensingMethod RangerMockSonar::getSensingMethod(void) {
  return SensingMethod::CONE;
}
