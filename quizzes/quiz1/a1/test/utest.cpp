#include "gtest/gtest.h"
#include <iostream> //Not needed usually, here just for debugging
#include <vector>
#include <algorithm>

//header files needed from our libraries
#include "../person.h"
#include "../processing.h"
using namespace std;


TEST (ClassTest, CreateObject) {
    Person alice("Alice",50,false);
    EXPECT_EQ(alice.getAge(), 50);
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
