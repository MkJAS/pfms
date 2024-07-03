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

TEST (AngleTest, LaserUnknownSonarFreeSonarUnknown){

    RangerMockLaser r1(180, 10, {1.0000,2.0000,-0.5236}, { 1.53474, 0.45429, 4.57736, 6.07856, 5.41957, 1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 5.07856, 5.41957, 1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 5.41957});
    RangerMockSonar r2(20, 0, {2.0000,-0.5000,0.1745}, { 6.93034});
    RangerMockSonar r3(20, 0, {-2.0000,-0.5000,0.1745}, { 5.93034});
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

TEST (AngleTest, LaserUknownSonarUknownSonarFree) {

    RangerMockLaser r1(180, 10, {1.0000,2.0000,-0.5236}, { 1.53474, 0.45429, 4.57736, 6.07856, 5.41957, 1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 5.07856, 5.41957, 1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 5.41957});
    RangerMockSonar r2(20, 0, {2.0000,-0.5000,-0.3491}, { 6.93034});
    RangerMockSonar r3(20, 0, {-2.0000,-0.5000,-0.6981}, { 5.93034});

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

TEST (AngleTest, LaserOccupiedSonarUknownSonarFree) {


    RangerMockLaser r1(180, 10, {1.0000,2.0000,-0.4363}, { 1.53474, 0.45429, 4.57736, 6.07856, 5.41957, 1.68538, 3.07755, 1.53474, 0.45429, 1.37736, 1.77856, 2.21957, 1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 4.67856, 5.41957});
    RangerMockSonar r2(20, 0, {2.0000,-0.5000,-0.3491}, { 6.93034});
    RangerMockSonar r3(20, 0, {-2.0000,-0.5000,-0.6981}, { 5.93034});

    std::vector<Cell*> cells;
    cells.push_back(new Cell());
    cells.at(0)->setCentre(1.2000,4.264);

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

