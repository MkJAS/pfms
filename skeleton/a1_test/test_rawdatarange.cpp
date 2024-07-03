#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "laser.h"
#include "rangerfusion.h"
#include "ranger.h"
#include "sonar.h"
#include "cell.h"

using namespace std;

//==================== HELPER FUNCTIONS ====================//
void testLimits(vector<double> stream, double min, double max) {
  for(auto measurement: stream) {
//    std::cout << measurement << ",";
    ASSERT_TRUE(measurement <= max);
    ASSERT_TRUE(measurement >= min);
  }
//  std::cout << std::endl;
}
//==================== HELPER FUNCTIONS ====================//

//==================== UNIT TEST START ====================//
TEST (SensingMethodTest, Sonar) {
  Sonar s;
  ASSERT_EQ(ranger::CONE, s.getSensingMethod());
}

TEST (SensingMethodTest, Laser) {
  Laser l;
  ASSERT_EQ(ranger::POINT, l.getSensingMethod());
}
//Test that data measurements are within limits
TEST (DataDistanceTest, Laser) {
  Laser l;

  //We arbitarily test the data 100 times as its randomly generated
  for(int i = 0; i < 100; i++) {
    testLimits(l.generateData(), 0.2, 8.0);
  }
}

TEST (DataDistanceTest, Sonar) {
  Sonar s;

  //We arbitarily test the data 100 times as its randomly generated
  for(int i = 0; i < 100; i++) {
    testLimits(s.generateData(), 0.2, 10.0);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
