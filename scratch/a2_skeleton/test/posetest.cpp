#include "gtest/gtest.h"
#include <iostream>

#include <vector>
#include <math.h>

// Student defined libraries, for instance
//#include "flightplanner.h"


#include "../dep/include/types.h"
#include "mockbogiepos.h"


//==================== UNIT TEST START ====================//

TEST(PoseTest, LocalToGlobal)
{
  /*
   Bogie positions = (2,2),(3,1),(-3,-2),(-3,3)
   Friendly pose = (0,0,0)

  */
  //Point ranger_bearing;
  std::vector<Point> correct = {{2000,2000},{3000,1000},{-3000,-2000},{-3000,3000}};
  std::vector<RangeBearingStamped> rbs = {{2828.4,0.7854,1},{3162.27766,0.3218,1},{3605.5513,3.7296,1},{4242.6407,2.3562,1}};
  Point friendxy = {0,0,0};
  double friend_orient = 0;
  Mockbogiepos m1;
  m1.getsimrbf(rbs,friendxy,friend_orient);
  std::vector<std::pair<Point,double>> testglobal = m1.getglobal();

  for(unsigned int i=0;i<rbs.size();i++)
  {
    EXPECT_NEAR(testglobal[i].first.x,correct[i].x,100);
    EXPECT_NEAR(testglobal[i].first.y,correct[i].y,100);
  }

  ////change friendly pose
  friendxy = {2000,0,0};
  friend_orient = M_PI/2;

  rbs = {{2000,0,2},{1414.2136,5.49779,2},{5385.16481,1.9513,2},{5830.9519,1.0304,1}};
  m1.getsimrbf(rbs,friendxy,friend_orient);
  testglobal = m1.getglobal();


  for(unsigned int i=0;i<rbs.size();i++)
  {
    EXPECT_NEAR(testglobal[i].first.x,correct[i].x,100);
    EXPECT_NEAR(testglobal[i].first.y,correct[i].y,100);
  }
}

TEST(PoseTest, GlobalToLocal)
{


  std::vector<Point> correct = {{2000,0},{1000,-1000},{-2000,5000},{3000,5000}};
  std::vector<Point> global = {{2000,2000},{3000,1000},{-3000,-2000},{-3000,3000}};
  Point friendxy = {2000,0,0};
  double friend_orient = M_PI/2;
  Mockbogiepos m1;
  
  for(unsigned int i=0;i<correct.size();i++)
  {
    Point xy = m1.global2local(friendxy,friend_orient,global[i]);
    EXPECT_NEAR(xy.x,correct[i].x,100);
    EXPECT_NEAR(xy.y,correct[i].y,100);
  }

  //change friendly pose
  friendxy = {2000,-2000,0};
  friend_orient = 1.2*M_PI;
  correct = {{-2351.11,-3236.068},{-2572.37,-1839.266},{4045.085,-2938.926},{1106.159,-6984.011}};
  for(unsigned int i=0;i<correct.size();i++)
  {
    Point xy = m1.global2local(friendxy,friend_orient,global[i]);
    EXPECT_NEAR(xy.x,correct[i].x,100);
    EXPECT_NEAR(xy.y,correct[i].y,100);
  }


}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
