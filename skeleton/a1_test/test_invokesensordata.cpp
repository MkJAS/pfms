#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "laser.h"
#include "rangerfusion.h"
#include "ranger.h"
#include "sonar.h"
#include "cell.h"

using namespace std;

TEST (NumMeasurementsTest, LaserAngRes30) {
  //Sensors:
  //  - (1) Laser {fov=180, angular_res=30, offset=0}
  //
  // Should produce 1 vector with 7 measurements. (no overlap)
  Laser l;
  l.setAngularResolution(30);
  l.setSensorPose({0,0,0});

  std::vector<RangerInterface *> sensors = { &l };
  auto rawdata = sensors.front()->generateData();

  ASSERT_EQ(7, rawdata.size());
}

TEST (NumMeasurementsTest, Sonar) {
  //Sensors:
  //  - (1) Sonar {fov=60, angular_res=20, offset=0}
  //
  // Should produce 1 vector with 3 measurements. (no overlap)
  Sonar s;
  s.setSensorPose({0,0,0});

  std::vector<RangerInterface *> sensors = { &s };
  auto rawdata = sensors.front()->generateData();

  ASSERT_EQ(1, rawdata.size());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
