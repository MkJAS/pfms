#include "gtest/gtest.h"

#include <vector>
#include <algorithm>
#include <cmath>

//header files needed from our libraries
#include "analysis.h"
#include "rectangle.h"
#include "circle.h"
#include "triangle.h"

// For marking the 3 below tests (from ShapesTest suite) will become one test, additional 3 will be used
TEST (ShapesTest, Rectangle) {
    Rectangle rectangle(2.0);
    EXPECT_EQ(rectangle.getType(),shape::Type::POLYGON);
    EXPECT_NEAR(rectangle.getArea(),4.0,1e-6);
    double width,height;
    rectangle.getWidthHeight(width,height);
    EXPECT_NEAR(width,2.0,1e-6);
    EXPECT_NEAR(height,2.0,1e-6);
    EXPECT_EQ(rectangle.getSides(),4);
}
TEST (ShapesTest, Circle) {
    Circle circle(2.0);
    EXPECT_EQ(circle.getType(),shape::Type::CIRCLE);
    EXPECT_NEAR(circle.getArea(),pow(2.0,2)*M_PI,1e-6);
    double width,height;
    circle.getWidthHeight(width,height);
    EXPECT_NEAR(width,4.0,1e-6);
    EXPECT_NEAR(height,4.0,1e-6);
    EXPECT_EQ(circle.getSides(),1);
}
TEST (ShapesTest, Triangle) {
        Triangle triangle(2.0);
        EXPECT_EQ(triangle.getType(),shape::Type::POLYGON);
        EXPECT_NEAR(triangle.getArea(),pow(2.0,2)*0.5,1e-6);
        double width,height;
        triangle.getWidthHeight(width,height);
        EXPECT_NEAR(width,2.0,1e-6);
        EXPECT_NEAR(height,2.0,1e-6);
        EXPECT_EQ(triangle.getSides(),3);
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
