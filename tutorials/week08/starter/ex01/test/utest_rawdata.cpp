#include "gtest/gtest.h"
#include <iostream> //Not needed usually, here just for debugging
#include <vector>
#include <algorithm>

//header files needed from our libraries
#include "laser.h"
#include "sonar.h"

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


//! TASK1A: What do the below two tests check?
//! Look at the marking criteria for Assignment 2 and identify what is the matching marking criteria

TEST (SensingMethodTest, Sonar) {
  Sonar s;
  ASSERT_EQ(ranger::CONE, s.getSensingMethod());
}

TEST (DataDistanceTest, Sonar) {
  Sonar s;

  //We arbitarily test the data 100 times as its randomly generated
  for(int i = 0; i < 100; i++) {
    testLimits(s.generateData(), 0.2, 16.0);
  }
}

//!TASK 1B: The above tests only check for Sonar and do not check for Laser
//! Implement the check for Laser (Sensing method and Sensing Range)
TEST (SensingMethodTest, Laser) 
{
  Laser l;
  ASSERT_EQ(ranger::POINT, l.getSensingMethod());
}
//Test that data measurements are within limits
TEST (DataDistanceTest, Laser) 
{
  Laser l;
  for(int i=0;i<50;i++)
  {
    testLimits(l.generateData(), 0.2, 8.0);
  }
}

//!TASK 1C: Implement a test for the fixed sensor parameters of Sonar
TEST (FixedParameters, Sonar) 
{
  Sonar s;
  ASSERT_EQ(20,s.getFieldOfView());
  ASSERT_EQ(20,s.getAngularResolution());
  ASSERT_EQ(0.2,s.getMinRange());
  ASSERT_EQ(16,s.getMaxRange());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
