#include "gtest/gtest.h"
#include <iostream> //Not needed usually, here just for debugging
#include <vector>
#include <algorithm>

//header files needed from our libraries
#include "../rectangle.h"
#include "../circle.h"
#include "../triangle.h"
using namespace std;


TEST (IntercectTest, Circle) {
    Circle circle(3.0);
    circle.setCentre(3.0,3.0);//Lets move centre to 3.0 3.0 so does not intercept
    ASSERT_FALSE(circle.checkIntercept(0,0));
    ASSERT_TRUE(circle.checkIntercept(1.0,1.0));
}

TEST (IntercectTest, Triangle) {
    Triangle triangle(3.0,3.0);
    triangle.setCentre(0.0,0.0);
    ASSERT_FALSE(triangle.checkIntercept(10,0));
    ASSERT_TRUE(triangle.checkIntercept(0,0));
}



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
