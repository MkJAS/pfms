#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "../laser.h"
#include "../rangerfusion.h"
#include "../ranger.h"
#include "../sonar.h"

using namespace std;
//==================== HELPER FUNCTIONS ====================//
void testLimits(vector<double> stream, double min, double max) {
  for(auto measurement: stream) {
    ASSERT_TRUE(measurement <= max);
    ASSERT_TRUE(measurement >= min);
  }
}
//==================== HELPER FUNCTIONS ====================//


//==================== UNIT TEST START ====================//
//Test that Ranger can not be instatiated
TEST (PureVirtualTest, Ranger) {
  Ranger r;
  r.setAngularResolution(10);
  r.setSensorPose({0,0,0});

  //We arbitarily test the data 2 times as its randomly generated
  for(int i = 0; i < 2; i++) {
    testLimits(r.generateData(), 0.2, 8.0);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
