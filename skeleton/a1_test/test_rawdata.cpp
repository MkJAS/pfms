#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

//Student defined libraries
#include "laser.h"
#include "rangerfusion.h"
#include "ranger.h"
#include "sonar.h"
#include "cell.h"

using namespace std;

//Test for correct number of measurements produced
TEST (NumMeasurementsTest, LaserAngRes10) {
  //Sensors:
  //  - (1) Laser {fov=180, angular_res=10, offset=0}
  //
  // Should produce 1 vector with 19 measurements. (no overlap)
  Laser l;
  l.setAngularResolution(10);
  l.setSensorPose({0,0,0});

  std::vector<RangerInterface *> sensors = { &l };
  auto rawdata = sensors.front()->generateData();

  ASSERT_EQ(19, rawdata.size());
}

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

//Test that multiple sensors produce a larger rawDataArray
TEST (MultipleSensorsTest, LaserAndSonarSimple) {
  //Sensors:
  //  - (1) Laser {fov=180, angular_res=30, offset=0}
  //  - (1) Sonar {fov=60, angular_res=20, offset=0}
  //
  // Produces two vectors, the first with 7 measurements and the second with
  // 1.
  Sonar s;
  s.setSensorPose({0,0,0});

  Laser l;
  l.setAngularResolution(30);
  l.setSensorPose({0,0,0});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0,0.5);

  std::vector<RangerInterface *> sensors = { &l, &s };
  RangerFusion rf(sensors);
  rf.setCells(cells);
  rf.grabAndFuseData();


  auto rawdata = rf.getRawRangeData();

  ASSERT_EQ(2, rawdata.size());
  ASSERT_EQ(7, rawdata[0].size());
  ASSERT_EQ(1, rawdata[1].size());
}

TEST (MultipleSensorsTest, LaserAndSonarWithOffset) {
  //Sensors:
  //  - (1) Laser {fov=180, angular_res=10, offset=0}
  //  - (1) Sonar {fov=60, angular_res=20, offset=90}
  //
  // Offset should not alter the raw data produced.
  // Produces two vectors, the first with 19 measurements and the second with
  // 3.
  Sonar s;
  s.setSensorPose({0,0,90*M_PI/180});

  Laser l;
  l.setAngularResolution(10);
  l.setSensorPose({0,0,0});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0,0.5);

  std::vector<RangerInterface *> sensors = { &l, &s };
  RangerFusion rf(sensors);
  rf.setCells(cells);
  rf.grabAndFuseData();
  rf.getRawRangeData();

  auto rawdata = rf.getRawRangeData();

  ASSERT_EQ(2, rawdata.size());
  ASSERT_EQ(19, rawdata[0].size());
  ASSERT_EQ(1, rawdata[1].size());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
