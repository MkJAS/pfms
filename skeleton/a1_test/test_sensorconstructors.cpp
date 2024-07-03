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
void testPose(ranger::SensorPose pose1, ranger::SensorPose pose2) {
    EXPECT_NEAR(pose1.x,pose2.x,1e-6);
    EXPECT_NEAR(pose1.y,pose2.y,1e-6);
    EXPECT_NEAR(pose1.theta,pose2.theta,1e-6);
}
//==================== HELPER FUNCTIONS ====================//

TEST (NumMeasurementsTest, LaserAngRes30) {

  Laser l;
  EXPECT_EQ(8.0, l.getMaxRange());
  EXPECT_EQ(0.2, l.getMinRange());
  EXPECT_EQ(180, l.getFieldOfView());
  EXPECT_EQ(ranger::SensingMethod::POINT,l.getSensingMethod());

  testPose({0, 0, 0},l.getSensorPose());
}

TEST (NumMeasurementsTest, Sonar) {

    Sonar s;
    EXPECT_EQ(10.0, s.getMaxRange());
    EXPECT_EQ(0.2, s.getMinRange());
    EXPECT_EQ(20, s.getFieldOfView());
    EXPECT_EQ(ranger::SensingMethod::CONE,s.getSensingMethod());

    testPose({0, 0, 0},s.getSensorPose());

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
