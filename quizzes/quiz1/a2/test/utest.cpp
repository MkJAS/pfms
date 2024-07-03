#include "gtest/gtest.h"
#include <iostream> //Not needed usually, here just for debugging
#include <vector>
#include <algorithm>

//header files needed from our libraries
#include "../rectangle.h"
using namespace std;


TEST (ClassTest, CreateObject) {
    Rectangle rectangle;
    rectangle.setHeightWidth(3.0,4.0);
    ASSERT_EQ(rectangle.getDescription(),"rectangle");
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
