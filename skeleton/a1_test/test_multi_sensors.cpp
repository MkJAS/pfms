#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "rangerfusion.h"

//Mock libraries for producing test data
#include "mock/rangermocklaser.h"
#include "mock/rangermocksonar.h"
#include "mock/mockrangerfusion.h"


TEST (MultiSensorTest,LaserUknownLaserOccupiedSonarUknownSonarFree) {


  RangerMockLaser r1(180, 30, 0, { 1.53474, 0.45429, 0.57736, 0.78560, 1.41957, 1.68538, 3.07755});
  RangerMockLaser r2(180, 30, -20, { 1.53474, 0.45429, 0.57736, 2.78560, 1.41957, 1.68538, 3.07755});
  RangerMockSonar r3(20, 0, 20, { 1.57947});
  RangerMockSonar r4(20, 0, -20, { 5.09880});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0.9500,2.6000);
  cells.at(0)->setSide(0.2000);

  std::vector<RangerInterface *> sensors = { &r1, &r2, &r3, &r4 };
  MockRangerFusion rf(sensors);
  rf.setCells(cells);

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(cell::OCCUPIED,cells.at(0)->getState());
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

