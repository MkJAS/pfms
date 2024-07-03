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

//==================== HELPER FUNCTIONS ====================//
void testPose(ranger::SensorPose pose1, ranger::SensorPose pose2) {
    EXPECT_NEAR(pose1.x,pose2.x,1e-6);
    EXPECT_NEAR(pose1.y,pose2.y,1e-6);
    EXPECT_NEAR(pose1.theta,pose2.theta,1e-6);
}
//==================== HELPER FUNCTIONS ====================//


TEST (RangerInterface, Laser) {

  ranger::SensorPose pose= {0,-5,35*M_PI/180};

  Laser l;
  l.setAngularResolution(30);
  l.setSensorPose(pose);

  ASSERT_EQ(30, l.getAngularResolution());

  ASSERT_EQ(false,l.setFieldOfView(90));

  testPose(pose,l.getSensorPose());

}

TEST (RangerInterface, Sonar) {

    ranger::SensorPose pose= {0,-5,35*M_PI/180};

    Sonar s;
    s.setSensorPose(pose);

    ASSERT_EQ(false,s.setFieldOfView(90));

    testPose(pose,s.getSensorPose());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
