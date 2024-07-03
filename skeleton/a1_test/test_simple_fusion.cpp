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

////////////////////////////////////////////////////////////////////////////////////////////////////////////


TEST (DataFusionTest, LaserUknownSonarOccupied) {

    RangerMockLaser r1(180, 30, {0.0,0.0,0.0}, {3.28502,3.06601,7.90626,0.49436,7.10431,7.32364,6.41023});
    RangerMockSonar r2(20, 20, {0.0,0.0,0.0}, {1.75965});
    std::vector<Cell*> cells;
    cells.push_back(new Cell());
    cells.at(0)->setCentre(0.0000,1.7597);
    cells.at(0)->setSide(0.2);

    std::vector<RangerInterface *> sensors = { &r1, &r2 };

    MockRangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::OCCUPIED,cells.at(0)->getState());
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST (DataFusionTest, LaserFreeSonarFreeSonarFree) {

  RangerMockLaser r1(180, 10, 0, { 1.53474, 0.45429, 4.57736, 7.07856, 5.41957, 1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 7.41957, 7.68538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 5.41957});
  RangerMockSonar r2(20, 0, 30, { 7.47947});
  RangerMockSonar r3(20, 0, 30, { 9.70988});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(-3.5000,6.2000);
  cells.at(0)->setSide(0.4);

  std::vector<RangerInterface *> sensors = { &r1, &r2, &r3 };
  MockRangerFusion rf(sensors);
  rf.setCells(cells);

  //Pull the raw data into internal storage variable
  rf.grabAndFuseData();
  EXPECT_EQ(cell::FREE,cells.at(0)->getState());
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST (DataFusionTest, LaserFreeSonarOccupied) {

    RangerMockLaser r1(180, 30, {0.0000,0.0000,0.0000}, {  3.28502, 3.06601, 7.90626, 6.49436, 7.10431, 7.32364, 6.41023});
    RangerMockSonar r2(20, 0, {0.0000,0.0000,0.0000}, {  4.26490});
    std::vector<Cell*> cells;
    cells.push_back(new Cell());
    cells.back()->setCentre(0.0000,4.2000);
    cells.back()->setSide(0.2000);

    std::vector<RangerInterface *> sensors = { &r1, &r2};
    MockRangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::OCCUPIED,cells.at(0)->getState());
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST (DataFusionTest,LaserUnknownSonarFreeSonarUnkown) {

  RangerMockLaser r1(180, 10, 5, { 1.53474, 0.45429, 4.57736, 7.07856, 5.41957, 1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 7.41957, 1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 5.41957});
  RangerMockSonar r2(20, 0, 20, { 9.47947});
  RangerMockSonar r3(20, 0, 20, { 5.09880});
  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(-2.5000,6.5000);
  cells.at(0)->setSide(0.2);

  std::vector<RangerInterface *> sensors = { &r1, &r2, &r3 };
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

