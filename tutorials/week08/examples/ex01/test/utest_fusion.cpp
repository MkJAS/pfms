#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "rangerfusion.h"

//Mock libraries for producing test data
#include "rangermocklaser.h"
#include "rangermocksonar.h"
#include "mockrangerfusion.h"

// Here is an example of a few tests, more than were discussed in tutorial
// As the takeaway from class was that we need more tests to isolate why
// Where the code fails.

// For the fusion tests we provide a mock fusion that draws the ranger and cells
// You will need

TEST (SimpleDataFusionTest, LaserOccupiedSonarFree) {
  RangerMockLaser r1(180, 30, {0,0,0}, {3.77902,7.71209,4.46508,4.26486,2.00644,4.01340,5.06767});
  RangerMockSonar r2(20, 20, {0,0,0}, {10.93034});

  //! Create fusion, supply cell, call the required functions for fusion to occur
  std::vector<RangerInterface *> sensors = { &r1, &r2 };
  MockRangerFusion rf(sensors);
  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0.0000,4.2649);
  cells.at(0)->setSide(0.2);
  rf.setCells(cells);
  rf.grabAndFuseData();

  //! Compare the anticipated outcome against the fusion outcome in comparsion (EXPECT_EQ)
  EXPECT_EQ(cell::OCCUPIED,cells.at(0)->getState());
}

TEST (AdvancedDataFusionTest, LaserOccupied) {

  RangerMockLaser r1(180, 10, {0,0,0}, { 1.53474, 0.45429, 0.57736, 0.78560, 1.41957, 1.68538, 3.07755, 1.53474, 1.45429, 4.57736, 7.07856, 7.41957, 1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 5.41957});


  //! Create cell as per the drawing
  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.back()->setCentre(0,0.4);
  cells.back()->setSide(0.2);

  //! Create fusion, supply cell, call the required functions for fusion to occur
  std::vector<RangerInterface *> sensors = { &r1 };
  MockRangerFusion rf(sensors);
  rf.setCells(cells);
  rf.grabAndFuseData();

  //! Compare the anticipated outcome against the fusion outcome in comparsion (EXPECT_EQ)
  EXPECT_EQ(cell::FREE,cells.at(0)->getState());
}



TEST (AdvancedDataFusionTest, LaserUknownSonarUnknownSonarUnkownSonarFree) {

  //! TASK3A: Examine how the Mock Laser is being constructed, see how offset, angular resolution and data are supplied in RangerMockLaser.
  RangerMockLaser r1(180, 10, {0,0,-10*M_PI/180}, { 1.53474, 0.45429, 0.57736, 0.78560, 1.41957, 1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 7.41957, 1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 5.41957});

  //! TASK3B: Construct three sonars, with data that matches the supplied drawing using the syntax for the RangerMockSonar
  RangerMockSonar r2(20, 20, {0,0,-110*M_PI/180}, {1.5});
  RangerMockSonar r3(20, 20, {0,0,-70*M_PI/180}, {4.0});
  RangerMockSonar r4(20, 20, {0,0,-20*M_PI/180}, {4.5});

  //! TASK3C: Create cell as per the drawing
  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.back()->setCentre(1.5,0.5);
  cells.back()->setSide(0.2);

  //! TASK3D: Create fusion, supply cell, call the required functions for fusion to occur
  std::vector<RangerInterface *> sensors = { &r1, &r2, &r3, &r4 };
  MockRangerFusion rf(sensors);
  rf.setCells(cells);
  rf.grabAndFuseData();

  //! TASK3E: Compare the anticipated outcome against the fusion outcome in comparsion (EXPECT_EQ)
  EXPECT_EQ(cell::FREE,cells.at(0)->getState());
}

TEST (AdvancedDataFusionTest, LaserOccupiedSonarUnknownSonarUnkownSonarFree) {

    RangerMockLaser r1(180, 10, {0,0,-10*M_PI/180}, { 1.53474, 0.45429, 0.57736, 0.78560, 1.41957, 1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 7.41957, 1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 5.41957});

    RangerMockSonar r2(20, 20, {0,0,-110*M_PI/180}, {1.5});
    RangerMockSonar r3(20, 20, {0,0,-70*M_PI/180}, {4.0});
    RangerMockSonar r4(20, 20, {0,0,-20*M_PI/180}, {4.5});

    std::vector<Cell*> cells;
    cells.push_back(new Cell());
    cells.back()->setCentre(0.75,0.27);
    cells.back()->setSide(0.2);

    std::vector<RangerInterface *> sensors = { &r1, &r2, &r3, &r4 };
    RangerFusion rf(sensors);
    rf.setCells(cells);
    rf.grabAndFuseData();
    rf.grabAndFuseData();

    EXPECT_EQ(cell::OCCUPIED,cells.at(0)->getState());
}



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
