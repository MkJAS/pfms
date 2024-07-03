#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>
#include "cell.h"

//Student defined libraries
#include "rangerfusion.h"

//Mock libraries for producing test data
#include "rangermocklaser.h"
#include "rangermocksonar.h"

TEST (AdvancedDataFusionTest, LaserUknownSonarUnknownSonarUnkownSonarFree) {

  //! TASK2A: Examine how the Mock Laser is being constructed, see how offset, angular resolution and data are supplied in RangerMockLaser.
  //RangerMockLaser(unsigned int fov, unsigned int res, ranger::SensorPose sensor_pose, std::vector<double> mockData);
  RangerMockLaser mockl(180, 10, {0,0,-10*M_PI/180}, { 1.53474, 0.45429, 0.57736, 0.78560, 1.41957, 
  1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 7.41957, 1.68538, 3.07755, 1.53474, 0.45429, 
  4.57736, 7.07856, 5.41957}); 
  //! TASK2B: Construct three sonars, with data that matches the supplied drawing using the syntax for the RangerMockSonar
  RangerMockSonar r2(20, 0, {0,0,-110*M_PI/180}, {1.5});
  RangerMockSonar r3(20, 0, {0,0,-70*M_PI/180}, {4.0});
  RangerMockSonar r4(20, 0, {0,0,-20*M_PI/180}, {4.5}); 
  //! TASK2C: Create cell as per the drawing
  Cell sell;
  sell.setCentre(1.5,0.5);
  //sell.setSide(0.2);
  //! TASK2D: Create fusion, supply cell, call the required functions for fusion to occur
  std::vector<RangerInterface*> rangers = {&mockl,&r2,&r3,&r4};
  std::vector<Cell*> cells = {&sell};
  RangerFusion fusion(rangers);
  fusion.setCells(cells);
  fusion.grabAndFuseData();
  //! TASK2E: Compare the anticipated outcome against the fusion outcome in comparsion (EXPECT_EQ)
  EXPECT_EQ(cell::FREE,cells[0]->getState());
}

TEST (AdvancedDataFusionTest, LaserOccupiedSonarUnknownSonarUnkownSonarFree) {

  RangerMockLaser mockl(180, 10, {0,0,-10*M_PI/180}, { 1.53474, 0.45429, 0.57736, 0.78560, 1.41957, 
  1.68538, 3.07755, 1.53474, 0.45429, 4.57736, 7.07856, 7.41957, 1.68538, 3.07755, 1.53474, 0.45429, 
  4.57736, 7.07856, 5.41957}); 
  RangerMockSonar r2(20, 0, {0,0,-110*M_PI/180}, {1.5});
  RangerMockSonar r3(20, 0, {0,0,-70*M_PI/180}, {4.0});
  RangerMockSonar r4(20, 0, {0,0,-20*M_PI/180}, {4.5}); 
  Cell sell;
  sell.setCentre(1.2,0.7);
  sell.setSide(1);
  std::vector<RangerInterface*> rangers = {&mockl,&r2,&r3,&r4};
  std::vector<Cell*> cells = {&sell};
  RangerFusion fusion(rangers);
  fusion.setCells(cells);
  fusion.grabAndFuseData();
  //! TASK2E: Compare the anticipated outcome against the fusion outcome in comparsion (EXPECT_EQ)
  EXPECT_EQ(cell::OCCUPIED,cells[0]->getState());

}



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
