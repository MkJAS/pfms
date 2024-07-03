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
TEST (ShapesTests, AllShapes) {
    {
        Rectangle rectangle(2.0);
        ASSERT_EQ(rectangle.getType(),shape::Type::POLYGON);
        EXPECT_NEAR(rectangle.getArea(),4.0,1e-6);
        double width,height;
        rectangle.getWidthHeight(width,height);
        EXPECT_NEAR(width,2.0,1e-6);
        EXPECT_NEAR(height,2.0,1e-6);
        EXPECT_EQ(rectangle.getSides(),4);
    }

    {
        Circle circle(2.0);
        ASSERT_EQ(circle.getType(),shape::Type::CIRCLE);
        EXPECT_NEAR(circle.getArea(),pow(2.0,2)*M_PI,1e-6);
        double width,height;
        circle.getWidthHeight(width,height);
        EXPECT_NEAR(width,4.0,1e-6);
        EXPECT_NEAR(height,4.0,1e-6);
        EXPECT_EQ(circle.getSides(),1);
    }

    {
        Triangle triangle(2.0);
        ASSERT_EQ(triangle.getType(),shape::Type::POLYGON);
        EXPECT_NEAR(triangle.getArea(),pow(2.0,2)*0.5,1e-6);
        double width,height;
        triangle.getWidthHeight(width,height);
        EXPECT_NEAR(width,2.0,1e-6);
        EXPECT_NEAR(height,2.0,1e-6);
        EXPECT_EQ(triangle.getSides(),3);
    }
}

TEST(CountShapes, AllShapes){
     std::vector<Shape*> shapes;
     shapes.push_back(new Rectangle(1.0));
     shapes.push_back(new Rectangle(1.0,2.0));
     shapes.push_back(new Circle(1.0));
     shapes.push_back(new Circle(2.0));
     shapes.push_back(new Triangle(1.0,2.0));
     shapes.push_back(new Circle(3.0));
     shapes.push_back(new Triangle(1.0,2.0));
     shapes.push_back(new Rectangle(1.0));
     shapes.push_back(new Rectangle(4.0));

     Line l({1.0, 5.0},{3.0, -5.0});
     Analysis analysis(shapes,l);

     std::vector<int> counters = analysis.detectShapes();
     ASSERT_EQ(counters.size(),3);
     EXPECT_EQ(counters.at(0),3);//Circles
     EXPECT_EQ(counters.at(1),2);//Triangles
     EXPECT_EQ(counters.at(2),4);//Rectangles
}


TEST(InterceptTest, Rectangle){

    std::vector<Shape*> shapes;
    for (double i = 2; i <= 4.0; i+=1.0){
      shapes.push_back(new Rectangle(i));
    }

    geometry_msgs::Point pt1{1.0, 5.0},pt2{3.0, -5.0};
    Line l(pt1,pt2);

    Analysis analysis(shapes,l);
    std::vector<bool> intersects = analysis.intersectsLine();

    ASSERT_EQ(shapes.size(),3);
    EXPECT_FALSE(intersects.at(0));
    EXPECT_FALSE(intersects.at(1));
    EXPECT_TRUE(intersects.at(2));
}


TEST(InterceptTest, Triangle){

    std::vector<Shape*> shapes;
    for (double i = 2; i <= 6.0; i+=2.0){
      shapes.push_back(new Triangle(i,i));
    }

    geometry_msgs::Point centre{2.0, 2.0};
    shapes.at(0)->setCentre(centre);

//    geometry_msgs::Point pt1{1.0, 5.0},pt2{3.0, -5.0};
    geometry_msgs::Point pt1{1.0, 5.0},pt2{2.0, -5.0};
    Line l(pt1,pt2);

    Analysis analysis(shapes,l);
    std::vector<bool> intersects = analysis.intersectsLine();

    ASSERT_EQ(shapes.size(),3);
    EXPECT_TRUE(intersects.at(0));
    EXPECT_TRUE(intersects.at(1));
    EXPECT_TRUE(intersects.at(2));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
