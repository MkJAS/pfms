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

TEST (SonarAreaTest, SonarsOverlap) {
  //Sensors:
  //  - (1) Sonar {fov=60, angular_res=20, offset=90}
  //  - (2) Sonar {fov=60, angular_res=20, offset=90}
  //
  // Offset should not alter the raw data produced.
  // Produces two vectors, the first with 19 measurements and the second with
  // 3.
  Sonar s1;
  s1.setSensorPose({0,0,90*M_PI/180});

  Sonar s2;
  s2.setSensorPose({0,0,90*M_PI/180});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0,0.5);

  std::vector<RangerInterface *> sensors = { &s1, &s2 };
  RangerFusion rf(sensors);
  rf.setCells(cells);
  rf.grabAndFuseData();
  rf.getRawRangeData();

  double scanningArea = rf.getScanningArea();
  //! https://www.omnicalculator.com/math/sector-area (R^2 * angle/ 2), angle in rad
  double expectedScanningArea = 2* (std::pow(16.0,2) * (20.0*(M_PI/180.0)) * 0.5);
  double tolerance =  2* (std::pow(0.2,2) * (20.0*(M_PI/180.0)) * 0.5); ; //! how about a tolerance

  tolerance = round( tolerance * 1000.0 ) / 1000.0;

  EXPECT_NEAR(scanningArea,expectedScanningArea,tolerance);

}

TEST (SonarAreaTest, SonarsPartOverlap) {
  //Sensors:
  //  - (1) Sonar {fov=60, angular_res=20, offset=5}
  //  - (2) Sonar {fov=60, angular_res=20, offset=-5}
  //
  // Offset should not alter the raw data produced.
  // Produces two vectors, the first with 19 measurements and the second with
  // 3.
  Sonar s1;
  s1.setSensorPose({0,0,5*M_PI/180});

  Sonar s2;
  s1.setSensorPose({0,0,-5*M_PI/180});

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0,0.5);

  std::vector<RangerInterface *> sensors = { &s1, &s2 };
  RangerFusion rf(sensors);
  rf.setCells(cells);
  rf.grabAndFuseData();
  rf.getRawRangeData();

  double scanningArea = rf.getScanningArea();
  //! https://www.omnicalculator.com/math/sector-area
  double expectedScanningArea = (std::pow(16.0,2) * (30.0*(M_PI/180.0)) * 0.5);
  double tolerance =  (std::pow(0.2,2) * (30.0*(M_PI/180.0)) * 0.5); ; //! how about a tolerance
  tolerance = round( tolerance * 1000.0 ) / 1000.0;

  EXPECT_NEAR(scanningArea,expectedScanningArea,tolerance);

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

