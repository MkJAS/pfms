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

  Laser r;
  r.setAngularResolution(30);
  r.setSensorPose(pose);
  r.setFieldOfView(180);
  r.getAngularResolution();
  r.getSensorPose();
  r.getFieldOfView();
  r.getMaxRange();
  r.getMinRange();
  r.getSensingMethod();


}

TEST (RangerInterface, Sonar) {

    ranger::SensorPose pose= {0,-5,35*M_PI/180};

    Sonar r;
    r.setAngularResolution(30);
    r.setSensorPose(pose);
    r.setFieldOfView(180);
    r.getAngularResolution();
    r.getSensorPose();
    r.getFieldOfView();
    r.getMaxRange();
    r.getMinRange();
    r.getSensingMethod();

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
