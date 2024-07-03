#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "rangerfusion.h"

//Mock libraries for producing test data
#include "mock/rangermocklaser.h"
#include "mock/rangermocksonar.h"


TEST (SingleSonarFusionTest, SonarUnknown) {

  RangerMockSonar r2(20, 20, 0, {1.55115});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0.0000,2.5141);
  cells.at(0)->setSide(0.2);

  std::vector<RangerInterface *> sensors = { &r2 };
  RangerFusion rf(sensors);
  rf.setCells(cells);

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(cell::UNKNOWN,cells.at(0)->getState());
}

TEST (SingleSonarFusionTest, SonarOccupied) {


  RangerMockSonar r2(20, 20, 0, {1.55115});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0.0000,1.55115);
  cells.at(0)->setSide(0.2);

  std::vector<RangerInterface *> sensors = { &r2 };
  RangerFusion rf(sensors);
  rf.setCells(cells);

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(cell::OCCUPIED,cells.at(0)->getState());
}

TEST (SingleSonarFusionTest, SonarFree) {


  RangerMockSonar r2(20, 20, 0, {1.55115});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0.0000,1.25);
  cells.at(0)->setSide(0.2);

  std::vector<RangerInterface *> sensors = { &r2 };
  RangerFusion rf(sensors);
  rf.setCells(cells);

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(cell::FREE,cells.at(0)->getState());
}

TEST (SingleSonarFusionTest, SonarFreeYaw) {

  RangerMockSonar r2(20, 0, 30, { 7.47947});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(-3.5000,6.2000);
  cells.at(0)->setSide(0.4);

  std::vector<RangerInterface *> sensors = { &r2 };
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
