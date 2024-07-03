#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

//Student defined libraries
#include "rangerfusion.h"

//Mock libraries for producing test data
#include "mock/rangermocklaser.h"
#include "mock/rangermocksonar.h"
#include "mock/mockrangerfusion.h"

//Laser Free, One Sonar Free, Other Sonar Occupied

TEST (SensorOrderingTest, LaserFreeSonarFreeSonarOccupied) {
// Fusion OCCUPIED (LASER FREE, SONAR FREE, SONAR OCCCUPIED);

    std::vector<Cell*> cells;
    cells.push_back(new Cell());
    cells.at(0)->setCentre(-3.7000,6.5000);
    cells.at(0)->setSide(0.2000);

  {
        RangerMockLaser r1(180, 10, {0,0,0}, { 1.53474, 0.45429, 4.57736, 7.07856, 5.41957, 1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 5.41957, 7.88538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 5.41957});
      RangerMockSonar r2(20, 0, {0,0,30*M_PI/180}, { 7.47947});
      RangerMockSonar r3(20, 0, {0,0,30*M_PI/180}, { 15.70988});


    std::vector<RangerInterface *> sensors = { &r1, &r2, &r3 };
    MockRangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::OCCUPIED,cells.at(0)->getState());

    auto rawdata = rf.getRawRangeData();

    ASSERT_EQ(3, rawdata.size());
    ASSERT_EQ(19, rawdata.at(0).size());
    ASSERT_EQ(1, rawdata.at(1).size());
    ASSERT_EQ(1, rawdata.at(2).size());
  }

  {
      RangerMockLaser r2(180, 10, {0,0,0}, { 1.53474, 0.45429, 4.57736, 7.07856, 5.41957, 1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 5.41957, 7.88538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 5.41957});
      RangerMockSonar r1(20, 0, {0,0,30*M_PI/180}, { 7.47947});
      RangerMockSonar r3(20, 0, {0,0,30*M_PI/180}, { 15.70988});

    std::vector<RangerInterface *> sensors = { &r1, &r2, &r3 };
    MockRangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::OCCUPIED,cells.at(0)->getState());

    auto rawdata = rf.getRawRangeData();

    ASSERT_EQ(3, rawdata.size());
    ASSERT_EQ(1, rawdata.at(0).size());
    ASSERT_EQ(19, rawdata.at(1).size());
    ASSERT_EQ(1, rawdata.at(2).size());
  }

  {
      RangerMockLaser r3(180, 10, 0, { 1.53474, 0.45429, 4.57736, 7.07856, 5.41957, 1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 5.41957, 7.88538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 5.41957});
      RangerMockSonar r1(20, 0, 30, { 7.47947});
      RangerMockSonar r2(20, 0, 30, { 15.70988});

    std::vector<RangerInterface *> sensors = { &r1, &r2, &r3 };
    MockRangerFusion rf(sensors);
    rf.setCells(cells);

    //Pull the raw data into internal storage variable
    rf.grabAndFuseData();
    EXPECT_EQ(cell::OCCUPIED,cells.at(0)->getState());

    auto rawdata = rf.getRawRangeData();

    ASSERT_EQ(3, rawdata.size());
    ASSERT_EQ(1, rawdata.at(0).size());
    ASSERT_EQ(1, rawdata.at(1).size());
    ASSERT_EQ(19, rawdata.at(2).size());
  }


}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

