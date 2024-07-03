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

TEST (AdvancedDataFusionTest, LaserUnknownSonarFreeSonarUnknown){

    RangerMockLaser r1(180, 10, {1.0000,2.0000,-0.5236}, {  1.53474, 0.45429, 4.57736, 6.07856, 5.41957, 1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 5.07856, 5.41957, 1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 5.41957});
    RangerMockSonar r2(20, 0, {2.0000,-0.5000,0.1745}, {  6.93034});
    RangerMockSonar r3(20, 0, {-2.0000,-0.5000,0.1745}, {  5.93034});
    std::vector<Cell*> cells;
    cells.push_back(new Cell());
    cells.at(0)->setCentre(1.2000,4.2649);
    cells.at(0)->setSide(0.2000);

    std::vector<RangerInterface *> sensors = { &r1, &r2, &r3 };

    MockRangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::FREE,cells.at(0)->getState());

}

///////////////////////////////////////////////////////////////////////

TEST (AdvancedDataFusionTest, LaserUnknownSonarFreeSonarOccupied) {

  RangerMockLaser r1(180, 10, 0, { 1.53474, 0.45429, 4.57736, 7.07856, 5.41957, 1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 5.41957, 1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 5.41957});
  RangerMockSonar r2(20, 0, 30, { 7.47947});
  RangerMockSonar r3(20, 0, 30, { 15.70988});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(-3.5000,6.5000);
  cells.at(0)->setSide(0.4);

  std::vector<RangerInterface *> sensors = { &r1, &r2, &r3 };
  MockRangerFusion rf(sensors);
  rf.setCells(cells);

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(cell::OCCUPIED,cells.at(0)->getState());
}

TEST (AdvancedDataFusionTest, LaserOccupiedSonarFreeSonarFree) {
//
// Fusion OCCUPIED (LASER Occupied, SONAR Free, SONAR Free);
//

  RangerMockLaser r1(180, 10, 0, { 1.53474, 0.45429, 4.57736, 7.07856, 5.41957, 1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 7.41957, 1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 5.41957});
  RangerMockSonar r2(20, 0, 2.000000e+01, { 10.47947});
  RangerMockSonar r3(20, 0, 2.000000e+01, { 15.70988});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(-2.5000,6.5000);
  cells.at(0)->setSide(2.0000);

  std::vector<RangerInterface *> sensors = { &r1, &r2, &r3 };
  MockRangerFusion rf(sensors);
  rf.setCells(cells);

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(cell::OCCUPIED,cells.at(0)->getState());
}


TEST (AdvancedDataFusionTest, LaserUknownSonarUnknownSonarUnkownSonarFree) {

  RangerMockLaser r1(180, 10, 0, { 1.53474, 0.45429, 0.57736, 0.78560, 1.41957, 1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 7.41957, 1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 5.41957});
  RangerMockSonar r2(20, 0, -110, { 1.57947});
  RangerMockSonar r3(20, 0, -20, { 5.09880});
  RangerMockSonar r4(20, 0, -70, { 4.50000});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(1.5000,0.5000);
  cells.at(0)->setSide(0.1000);

  std::vector<RangerInterface *> sensors = { &r1, &r2, &r3 ,&r4 };
  MockRangerFusion rf(sensors);
  rf.setCells(cells);

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(cell::FREE,cells.at(0)->getState());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

