#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <thread>

//Student defined libraries
#include "laser.h"
#include "rangerfusion.h"
#include "ranger.h"
#include "sonar.h"
#include "cell.h"

using namespace std;

//==================== HELPER FUNCTIONS ====================//
long int computeRefreshRate(int samples,RangerFusion rf){

  int count =0;
  auto start = std::chrono::steady_clock::now();
  while (count++ < samples)
  {
    rf.grabAndFuseData();
  }

  auto end = std::chrono::steady_clock::now();
  auto difference = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  return difference.count() / count;
}


//Test that multiple sensors produce a larger rawDataArray
TEST (MultipleSensorsTest, LaserAndSonarSimple) {
  //Sensors:
  //  - (1) Laser {fov=180, angular_res=30, offset=0}
  //  - (1) Sonar {fov=60, angular_res=20, offset=0}
  //
  // Produces two vectors, the first with 7 measurements and the second with
  // 1.
  Sonar s;
  s.setAngularOffset(0);

  Laser l;
  l.setAngularResolution(30);
  l.setAngularOffset(0);

  std::vector<Cell*> cells;
  cells.push_back(new Cell());
  cells.at(0)->setCentre(0,0.5);

  std::vector<RangerInterface *> sensors = { &l, &s };
  RangerFusion rf(sensors);
  rf.setCells(cells);

  int samples=5;

  long int rateMs = computeRefreshRate(samples,rf);

  ASSERT_NEAR(rateMs,5000,20);

}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
