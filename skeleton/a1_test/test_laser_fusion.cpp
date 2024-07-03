#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "rangerfusion.h"

//Mock libraries for producing test data
#include "mock/rangermocklaser.h"
#include "mock/rangermocksonar.h"


////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST (SingleLaserFusionTest, LaserOccupied) {

  RangerMockLaser r1(180, 30, 0, {5.74748, 1.92962, 1.11586, 2.51407, 2.68647, 3.50850, 4.16129});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0.0000,2.5141);
  cells.at(0)->setSide(0.2);

  std::vector<RangerInterface *> sensors = { &r1};
  RangerFusion rf(sensors);
  rf.setCells(cells);

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(cell::OCCUPIED,cells.at(0)->getState());
}

TEST (SingleLaserFusionTest, LaserUknown) {


  RangerMockLaser r1(180, 30, 0, {3.77902,7.71209,4.46508,4.26486,2.00644,4.01340,5.06767});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0.0000,5.1492);
  cells.at(0)->setSide(0.2);

  std::vector<RangerInterface *> sensors = { &r1};
  RangerFusion rf(sensors);
  rf.setCells(cells);

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(cell::UNKNOWN,cells.at(0)->getState());
}


TEST (SingleLaserFusionTest, LaserFree30degRes) {

  RangerMockLaser r1(180, 30, 0, {6.27660,5.77729,7.24902,7.14920,2.80647,5.65022,1.74292});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0.0000,5.1492);
  cells.at(0)->setSide(0.2);

  std::vector<RangerInterface *> sensors = { &r1};
  RangerFusion rf(sensors);
  rf.setCells(cells);

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(cell::FREE,cells.at(0)->getState());
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST (SingleLaserFusionTest, LaserFree10degRes) {

  RangerMockLaser r1(180, 10, 0, { 1.53474, 0.45429, 4.57736, 7.07856, 5.41957, 1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 7.41957, 7.68538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 5.41957});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(-3.5000,6.2000);
  cells.at(0)->setSide(0.4);

  std::vector<RangerInterface *> sensors = { &r1 };
  RangerFusion rf(sensors);
  rf.setCells(cells);

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(cell::FREE,cells.at(0)->getState());
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
