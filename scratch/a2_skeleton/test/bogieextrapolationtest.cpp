#include "gtest/gtest.h"
#include <iostream>

#include <vector>

// Student defined libraries, for instance
#include "mockiebogieposAdv.h"
#include "../controlAdv.h"





//==================== UNIT TEST START ====================//


TEST(BogieTest, DetermineVelocity)
{
  //getvel(std::pair<int,int>& it, std::vector<std::pair<Point,double>>& t1,std::vector<std::pair<Point,double>>& t0,std::vector<double>& times1, std::vector<double>& times0,double& pose)
  MockbogieposAdv m1;
  //create set of readings with times at times0
  std::vector<std::pair<Point,double>> t0 = {{{4579,206,0},0},{{2416,3969,0},0},{{3398,-3322,0},0},{{-2035,4281,0},0}};
  //create second set of readings after a period of time with times at times1
  std::vector<std::pair<Point,double>> t1 = {{{4450,239,0},0},{{2328,3915,0},0},{{3407,-3275,0},0},{{-2056,4230,0},0}};
  std::vector<double> times0 = {518,518,518,518}; //time stamps for first set
  std::vector<double> times1 = {762,762,762,762}; //time stamps for second set
  std::vector<velocity> vels;
  velocity vel;
  double pose; //leftover func paramter from actual class. Disregard

   std::vector<std::pair<int,int>> markers = {{0,0},{1,1},{2,2},{3,3}}; //match the elements in t0 with those in t1 by index

  for(auto it:markers)
  {
    vel = m1.getvel(it,t1,t0,times1,times0,pose);
    vels.push_back(vel);
  }
  //Test velocities in X direction
  ASSERT_NEAR(vels[0].vx,-528.69,10);
  ASSERT_NEAR(vels[1].vx,-360.66,10);
  ASSERT_NEAR(vels[2].vx,36.89,10);
  ASSERT_NEAR(vels[3].vx,-86.07,10);
  //Test velocities in Y direction
  ASSERT_NEAR(vels[0].vy,135.25,10);
  ASSERT_NEAR(vels[1].vy,-221.31,10);
  ASSERT_NEAR(vels[2].vy,192.62,10);
  ASSERT_NEAR(vels[3].vy,-209.02,10);
  //Test complete linear velocity
  ASSERT_NEAR(abs(vels[0].vx/cos(vels[0].m)),545.03,10);
  ASSERT_NEAR(abs(vels[1].vx/cos(vels[1].m)),423.04,10);
  ASSERT_NEAR(abs(vels[2].vx/cos(vels[2].m)),196.15,10);
  ASSERT_NEAR(abs(vels[3].vx/cos(vels[3].m)),222.29,10);



}

TEST(BogieTest, ExtrapolationInTime)
{
  MockbogieposAdv m1;
  std::vector<std::pair<Point,double>> t0 = {{{4579,206,0},0},{{2416,3969,0},0},{{3398,-3322,0},0},{{-2035,4281,0},0}};
  std::vector<std::pair<Point,double>> t1 = {{{4450,239,0},0},{{2328,3915,0},0},{{3407,-3275,0},0},{{-2056,4230,0},0}};
  std::vector<double> times0 = {518,518,518,518}; 
  std::vector<double> times1 = {762,762,762,762}; 
  std::vector<velocity> vels;
  velocity vel;
  double pose;
  std::vector<std::pair<int,int>> markers = {{0,0},{1,1},{2,2},{3,3}}; 
  for(auto it:markers)
  {
    vel = m1.getvel(it,t1,t0,times1,times0,pose);
    vels.push_back(vel);
  }
  ControlAdv c1;
  double tx;
  double ty;
  std::vector<Point> xyT;
  //TEST FOR 1 second after calculating the velocity
  for(auto it:vels)
  {
    double T = it.time/1000 + 1;
    c1.extrapbogiexy(it,T,ty,tx);
    xyT.push_back({tx,ty});
  } 
  ASSERT_NEAR(xyT[0].x,3921.31,10);
  ASSERT_NEAR(xyT[1].x,1967.34,10);
  ASSERT_NEAR(xyT[2].x,3443.89,10);
  ASSERT_NEAR(xyT[3].x,-2142.07,10);
  

}



int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
