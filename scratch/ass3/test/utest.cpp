#include <gtest/gtest.h>
#include <climits>

#include <ros/package.h>

//These allow us to inspect bags of data
#include <rosbag/bag.h>
#include <rosbag/view.h>

//We include our header file
#include "../src/grid_processing.h"
#include "../src/topic_handler.h"
#include "../src/utilities.h"
#include "fnf_mock.h"

#include <sstream>
#include <iostream>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h" 

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "project_setup/RequestGoal.h"


//==================== HELPER FUNCTIONS ====================//
void getData(rosbag::Bag& bag, nav_msgs::OccupancyGrid::ConstPtr& grid,
  sensor_msgs::LaserScan::ConstPtr& scan,
  nav_msgs::Odometry::ConstPtr& r0,
  nav_msgs::Odometry::ConstPtr& r1)
  {
      for(rosbag::MessageInstance const m: rosbag::View(bag))
      {
        if(m.getTopic() == "/robot_0/odom")
        {
            r0 = m.instantiate<nav_msgs::Odometry>();
        }
        if(r0 != nullptr)
        {
            break;
        }
      }
    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
      if(m.getTopic() == "/robot_1/odom")
      {
          r1 = m.instantiate<nav_msgs::Odometry>();
      }
      if(r1 != nullptr)
      {
          break;
      }
    }
    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
      if(m.getTopic() == "/robot_0/base_scan")
      {
          scan = m.instantiate<sensor_msgs::LaserScan>();
      }
      if(scan != nullptr)
      {
          break;
      }
    }
    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
      if(m.getTopic() == "/local_map/local_map")
      {
          grid = m.instantiate<nav_msgs::OccupancyGrid>();
      }
      if(grid != nullptr)
      {
          break;
      }
    }

  }

//==================== HELPER FUNCTIONS ====================//

//First lets test that the gridprocessing functions actually work so that we can use them in later tests
TEST(GridprocessingTest,GridTests)
{
  std::string path = ros::package::getPath("ass3");
  path += "/test/samples/";
  std::string file = path + "gridtest.bag";
  rosbag::Bag bag;
  bag.open(file);  // BagMode is Read by default

  nav_msgs::OccupancyGrid::ConstPtr grid;
  sensor_msgs::LaserScan::ConstPtr scan;
  nav_msgs::Odometry::ConstPtr r0;
  nav_msgs::Odometry::ConstPtr r1;
  getData(bag,grid,scan,r0,r1);  
  bag.close();
  FnF_mock fnf1;
  FnFData data;
  data.laserscan = *scan;
  data.pose0 = r0->pose.pose;
  data.pose1 = r1->pose.pose;
  fnf1.setData(data,*grid);

  geometry_msgs::Point zero;
  geometry_msgs::Point goal_local;
  zero.x = data.pose0.position.x; //known to be set at origin so close to (0,0)
  zero.y = data.pose0.position.y;
  goal_local.x = -2 - zero.x;
  goal_local.y = 0 - zero.y;
  GridProcessing gridtest(*grid);
  bool test1 = gridtest.checkConnectivity(zero,goal_local); //known to be free

  goal_local.x = 4 - zero.x;  //known to be blocked
  goal_local.y = 0 - zero.x;
  bool test2 = gridtest.checkConnectivity(zero,goal_local); 

  goal_local.x = 3.2 - zero.x;
  goal_local.y = 3.8 - zero.y;  //should be blocked
  bool test3 = gridtest.checkConnectivity(zero,goal_local);

  goal_local.x = -3.6 - zero.x;
  goal_local.y = 2.97 - zero.y;  //should be free
  bool test4 = gridtest.checkConnectivity(zero,goal_local);

  ASSERT_TRUE(test1);
  ASSERT_FALSE(test2);
  ASSERT_FALSE(test3);
  ASSERT_TRUE(test4);

}

TEST(InsightTests,InsightTest)
{
  {
    std::string path = ros::package::getPath("ass3");
    path += "/test/samples/";
    std::string file = path + "Insight1.bag";
    rosbag::Bag bag;
    bag.open(file);  // BagMode is Read by default

    nav_msgs::OccupancyGrid::ConstPtr grid;
    sensor_msgs::LaserScan::ConstPtr scan;
    nav_msgs::Odometry::ConstPtr r0;
    nav_msgs::Odometry::ConstPtr r1;
    getData(bag,grid,scan,r0,r1);  
    bag.close();
    FnF_mock fnf1;
    FnFData data;
    data.laserscan = *scan;
    data.pose0 = r0->pose.pose;
    data.pose1 = r1->pose.pose;
    fnf1.setData(data,*grid);

    ASSERT_TRUE(fnf1.IsinLoS());
  }

  {
    std::string path = ros::package::getPath("ass3");
    path += "/test/samples/";
    std::string file = path + "behind_not_insight.bag";
    rosbag::Bag bag;
    bag.open(file);  // BagMode is Read by default
    nav_msgs::OccupancyGrid::ConstPtr grid;//We need to have a pointer to extract the grid
    sensor_msgs::LaserScan::ConstPtr scan;
    nav_msgs::Odometry::ConstPtr r0;
    nav_msgs::Odometry::ConstPtr r1;
    getData(bag,grid,scan,r0,r1);
    bag.close();
    FnF_mock fnf1;
    FnFData data;
    data.laserscan = *scan;
    data.pose0 = r0->pose.pose;
    data.pose1 = r1->pose.pose;
    fnf1.setData(data,*grid);

    ASSERT_FALSE(fnf1.IsinLoS());
  }

  {
    std::string path = ros::package::getPath("ass3");
    path += "/test/samples/";
    std::string file = path + "garage_not_insight.bag";
    rosbag::Bag bag;
    bag.open(file);  // BagMode is Read by default
    nav_msgs::OccupancyGrid::ConstPtr grid;//We need to have a pointer to extract the grid
    sensor_msgs::LaserScan::ConstPtr scan;
    nav_msgs::Odometry::ConstPtr r0;
    nav_msgs::Odometry::ConstPtr r1;
    getData(bag,grid,scan,r0,r1);
    bag.close();
    FnF_mock fnf1;
    FnFData data;
    data.laserscan = *scan;
    data.pose0 = r0->pose.pose;
    data.pose1 = r1->pose.pose;
    fnf1.setData(data,*grid);

    ASSERT_FALSE(fnf1.IsinLoS());
  }

  {
    std::string path = ros::package::getPath("ass3");
    path += "/test/samples/";
    std::string file = path + "r0_behind_r1.bag";
    rosbag::Bag bag;
    bag.open(file);  // BagMode is Read by default
    nav_msgs::OccupancyGrid::ConstPtr grid;//We need to have a pointer to extract the grid
    sensor_msgs::LaserScan::ConstPtr scan;
    nav_msgs::Odometry::ConstPtr r0;
    nav_msgs::Odometry::ConstPtr r1;
    getData(bag,grid,scan,r0,r1);
    bag.close();
    FnF_mock fnf1;
    FnFData data;
    data.laserscan = *scan;
    data.pose0 = r0->pose.pose;
    data.pose1 = r1->pose.pose;
    fnf1.setData(data,*grid);

    ASSERT_TRUE(fnf1.IsinLoS());
  }
}

TEST(Following,BasicMode)
{
  std::string path = ros::package::getPath("ass3");
  path += "/test/samples/";
  std::string file = path + "Insight1.bag";
  rosbag::Bag bag;
  bag.open(file);  // BagMode is Read by default
  nav_msgs::OccupancyGrid::ConstPtr grid;//We need to have a pointer to extract the grid
  sensor_msgs::LaserScan::ConstPtr scan;
  nav_msgs::Odometry::ConstPtr r0;
  nav_msgs::Odometry::ConstPtr r1;
  getData(bag,grid,scan,r0,r1);
  bag.close();
  FnF_mock fnf1;
  FnFData data;
  data.laserscan = *scan;
  data.pose0 = r0->pose.pose;
  data.pose1 = r1->pose.pose;
  fnf1.setData(data,*grid);

  geometry_msgs::Point goal = getpointaroundR1(data.pose0.position,data.pose1.position);
  geometry_msgs::Pose temp;
  temp.position = data.pose1.position;
        

  double yaw = atan2((data.pose1.position.y-data.pose0.position.y),(data.pose1.position.x-data.pose0.position.x));
  if(yaw<0){yaw = yaw + 2*M_PI;}
  YawtoQuat(yaw,temp);
  temp.position = goal;
  geometry_msgs::Pose current = data.pose0;
  geometry_msgs::Pose current1 = data.pose1;
  Pathpacket packet = {current,current1,temp};
  geometry_msgs::PoseArray pose = fnf1.getpath(packet);

  //Since BASIC mode works by finding a point within a 1m radius around r1, the tests should reflect that distance
  ASSERT_NEAR(pose.poses[0].position.x,1.63,1); //r1 at (1.63,-1.02)
  ASSERT_NEAR(pose.poses[0].position.y,-1,1);

}

TEST(Following,AdvMode)
{
  std::string path = ros::package::getPath("ass3");
  path += "/test/samples/";
  std::string file = path + "r0_behind_r1.bag";
  rosbag::Bag bag;
  bag.open(file);  // BagMode is Read by default
  nav_msgs::OccupancyGrid::ConstPtr grid;//We need to have a pointer to extract the grid
  sensor_msgs::LaserScan::ConstPtr scan;
  nav_msgs::Odometry::ConstPtr r0;
  nav_msgs::Odometry::ConstPtr r1;
  getData(bag,grid,scan,r0,r1);
  bag.close();
  FnF_mock fnf1;
  FnFData data;
  data.laserscan = *scan;
  data.pose0 = r0->pose.pose;
  data.pose1 = r1->pose.pose;
  fnf1.setData(data,*grid);

  geometry_msgs::Point goal = getpointbehindR1(data.pose1);
  geometry_msgs::Pose temp;
  temp.position = data.pose1.position;
        

  double yaw = atan2((data.pose1.position.y-data.pose0.position.y),(data.pose1.position.x-data.pose0.position.x));
  if(yaw<0){yaw = yaw + 2*M_PI;}
  YawtoQuat(yaw,temp);
  temp.position = goal;
  geometry_msgs::Pose current = data.pose0;
  geometry_msgs::Pose current1 = data.pose1;
  Pathpacket packet = {current,current1,temp};
  geometry_msgs::PoseArray pose = fnf1.getpath(packet);

  //r1 is at (-1.43,-0.73) and is facing North
  ASSERT_NEAR(pose.poses[0].position.x,-1.43,0.2); //should be similar to r1
  ASSERT_NEAR(pose.poses[0].position.y,-1.4,0.2);    //should be about 0.7 behind r1

}

TEST(FollowingTest2,BasicMode)
{
  std::string path = ros::package::getPath("ass3");
  path += "/test/samples/";
  std::string file = path + "basic_pose.bag";
  rosbag::Bag bag;
  bag.open(file);  // BagMode is Read by default
  nav_msgs::OccupancyGrid::ConstPtr grid;//We need to have a pointer to extract the grid
  sensor_msgs::LaserScan::ConstPtr scan;
  nav_msgs::Odometry::ConstPtr r0;
  nav_msgs::Odometry::ConstPtr r1;
  getData(bag,grid,scan,r0,r1);
  bag.close();
  FnF_mock fnf1;
  FnFData data;
  data.laserscan = *scan;
  data.pose0 = r0->pose.pose;
  data.pose1 = r1->pose.pose;
  fnf1.setData(data,*grid);

  geometry_msgs::Point goal = getpointaroundR1(data.pose0.position,data.pose1.position);
  geometry_msgs::Pose temp;
  temp.position = data.pose1.position;
        

  double yaw = atan2((data.pose1.position.y-data.pose0.position.y),(data.pose1.position.x-data.pose0.position.x));
  if(yaw<0){yaw = yaw + 2*M_PI;}
  YawtoQuat(yaw,temp);
  temp.position = goal;
  geometry_msgs::Pose current = data.pose0;
  geometry_msgs::Pose current1 = data.pose1;
  Pathpacket packet = {current,current1,temp};
  geometry_msgs::PoseArray pose = fnf1.getpath(packet);

  //Since BASIC mode works by findind the closest point to r1 (1.06,0.12) from r0, which would be near (0,0);
  ASSERT_NEAR(pose.poses[0].position.x,0,0.1);
  ASSERT_NEAR(pose.poses[0].position.y,0.1,0.1);

}

TEST(FollowingTest2,AdvMode)
{
  std::string path = ros::package::getPath("ass3");
  path += "/test/samples/";
  std::string file = path + "basic_pose.bag";
  rosbag::Bag bag;
  bag.open(file);  // BagMode is Read by default
  nav_msgs::OccupancyGrid::ConstPtr grid;//We need to have a pointer to extract the grid
  sensor_msgs::LaserScan::ConstPtr scan;
  nav_msgs::Odometry::ConstPtr r0;
  nav_msgs::Odometry::ConstPtr r1;
  getData(bag,grid,scan,r0,r1);
  bag.close();
  FnF_mock fnf1;
  FnFData data;
  data.laserscan = *scan;
  data.pose0 = r0->pose.pose;
  data.pose1 = r1->pose.pose;
  fnf1.setData(data,*grid);

  geometry_msgs::Point goal = getpointbehindR1(data.pose1);
  geometry_msgs::Pose temp;
  temp.position = data.pose1.position;
        

  double yaw = atan2((data.pose1.position.y-data.pose0.position.y),(data.pose1.position.x-data.pose0.position.x));
  if(yaw<0){yaw = yaw + 2*M_PI;}
  YawtoQuat(yaw,temp);
  temp.position = goal;
  geometry_msgs::Pose current = data.pose0;
  geometry_msgs::Pose current1 = data.pose1;
  Pathpacket packet = {current,current1,temp};
  geometry_msgs::PoseArray pose = fnf1.getpath(packet);

  //In adv mode the point should be roughly 0.7 behind r1, which is (1.02,0.82);
  ASSERT_NEAR(pose.poses[0].position.x,1.02,0.1);
  ASSERT_NEAR(pose.poses[0].position.y,0.82,0.1);

}

TEST(Aroundcorner,HorizontalCorridor)
{
  
  std::string path = ros::package::getPath("ass3");
  path += "/test/samples/";
  std::string file = path + "around_corner3.bag";
  rosbag::Bag bag;
  bag.open(file);  // BagMode is Read by default
  nav_msgs::OccupancyGrid::ConstPtr grid;//We need to have a pointer to extract the grid
  sensor_msgs::LaserScan::ConstPtr scan;
  nav_msgs::Odometry::ConstPtr r0;
  nav_msgs::Odometry::ConstPtr r1;
  getData(bag,grid,scan,r0,r1);
  bag.close();
  FnF_mock fnf1;
  FnFData data;
  data.laserscan = *scan;
  data.pose0 = r0->pose.pose;
  data.pose1 = r1->pose.pose;
  fnf1.setData(data,*grid);

  //Test Line of Sight
  ASSERT_TRUE(fnf1.IsinLoS());

  geometry_msgs::Point goal = getpointaroundR1(data.pose0.position,data.pose1.position);
  geometry_msgs::Pose temp;
  temp.position = data.pose1.position;
        

  double yaw = atan2((data.pose1.position.y-data.pose0.position.y),(data.pose1.position.x-data.pose0.position.x));
  if(yaw<0){yaw = yaw + 2*M_PI;}
  YawtoQuat(yaw,temp);
  temp.position = goal;
  geometry_msgs::Pose current = data.pose0;
  geometry_msgs::Pose current1 = data.pose1;
  Pathpacket packet = {current,current1,temp};
  geometry_msgs::PoseArray pose = fnf1.getpath(packet);

  geometry_msgs::Point zero;
  geometry_msgs::Point r1_local;

  zero.x = pose.poses[0].position.x - current.position.x;
  zero.y = pose.poses[0].position.y - current.position.y;

  r1_local.x = data.pose1.position.x - current.position.x;
  r1_local.y = data.pose1.position.y - current.position.y;

  //check that goal is reachable from the calculated point
  GridProcessing proc(*grid);
  bool check = proc.checkConnectivity(zero,r1_local);
  ASSERT_TRUE(check);

  //We know that calculated pose should be somewhere along the same y axis as robot1 in order to be insight
  //which is (-4.06,2.7) and the x should be similar to robots0 current x which is -4.07

  ASSERT_NEAR(pose.poses[0].position.x,-4.07,0.2); //test for calculated pose
  ASSERT_NEAR(pose.poses[0].position.y,2.7,0.2);


}

TEST(Aroundcorner,VerticalCorridor)
{
  
  std::string path = ros::package::getPath("ass3");
  path += "/test/samples/";
  std::string file = path + "around_corner2.bag";
  rosbag::Bag bag;
  bag.open(file);  // BagMode is Read by default
  nav_msgs::OccupancyGrid::ConstPtr grid;//We need to have a pointer to extract the grid
  sensor_msgs::LaserScan::ConstPtr scan;
  nav_msgs::Odometry::ConstPtr r0;
  nav_msgs::Odometry::ConstPtr r1;
  getData(bag,grid,scan,r0,r1);
  bag.close();
  FnF_mock fnf1;
  FnFData data;
  data.laserscan = *scan;
  data.pose0 = r0->pose.pose;
  data.pose1 = r1->pose.pose;
  fnf1.setData(data,*grid);

  //Check line of sight
  ASSERT_TRUE(fnf1.IsinLoS());

  geometry_msgs::Point goal = getpointaroundR1(data.pose0.position,data.pose1.position);
  geometry_msgs::Pose temp;
  temp.position = data.pose1.position;
        

  double yaw = atan2((data.pose1.position.y-data.pose0.position.y),(data.pose1.position.x-data.pose0.position.x));
  if(yaw<0){yaw = yaw + 2*M_PI;}
  YawtoQuat(yaw,temp);
  temp.position = goal;
  geometry_msgs::Pose current = data.pose0;
  geometry_msgs::Pose current1 = data.pose1;
  Pathpacket packet = {current,current1,temp};
  geometry_msgs::PoseArray pose = fnf1.getpath(packet);

  geometry_msgs::Point zero;
  geometry_msgs::Point r1_local;

  zero.x = pose.poses[0].position.x - current.position.x;
  zero.y = pose.poses[0].position.y - current.position.y;

  r1_local.x = data.pose1.position.x - current.position.x;
  r1_local.y = data.pose1.position.y - current.position.y;

  GridProcessing proc(*grid);
  bool check = proc.checkConnectivity(zero,r1_local);
  ASSERT_TRUE(check);
  //We know that calculated pose should be somewhere along the same x axis as robot1 in order to be insight
  //which is (-10.06,4.56) and the y should be similar to robots0 current y which is around 3.5

  ASSERT_NEAR(pose.poses[0].position.x,-10,0.2); //test for calculated pose
  ASSERT_NEAR(pose.poses[0].position.y,3.5,0.2);
}


TEST(StandBeside,behind)
{
  std::string path = ros::package::getPath("ass3");
  path += "/test/samples/";
  std::string file = path + "stand_beside.bag";
  rosbag::Bag bag;
  bag.open(file);  // BagMode is Read by default
  nav_msgs::OccupancyGrid::ConstPtr grid;//We need to have a pointer to extract the grid
  sensor_msgs::LaserScan::ConstPtr scan;
  nav_msgs::Odometry::ConstPtr r0;
  nav_msgs::Odometry::ConstPtr r1;
  getData(bag,grid,scan,r0,r1);
  bag.close();
  FnF_mock fnf1;
  FnFData data;
  data.laserscan = *scan;
  data.pose0 = r0->pose.pose;
  data.pose1 = r1->pose.pose;
  fnf1.setData(data,*grid);
  geometry_msgs::PoseArray array = fnf1.standbeside();
  //need to get the point to the right of r1
  geometry_msgs::Point right; //point to the right taken from looking at sim
  right.x = -0.7;             //we have set the code to move r0 about 1 unit to the side of r0, r1 is at x=0.3, 0.3-1 = -0.7
  right.y = -3.81;            //y remains the same
  
  EXPECT_NEAR(array.poses[0].position.x,right.x,0.2);
  EXPECT_NEAR(array.poses[0].position.y,right.y,0.2);
}

TEST(StandBeside,Rightsideblocked)
{
  std::string path = ros::package::getPath("ass3");
  path += "/test/samples/";
  std::string file = path + "standtoleft.bag";
  rosbag::Bag bag;
  bag.open(file);  // BagMode is Read by default
  nav_msgs::OccupancyGrid::ConstPtr grid;//We need to have a pointer to extract the grid
  sensor_msgs::LaserScan::ConstPtr scan;
  nav_msgs::Odometry::ConstPtr r0;
  nav_msgs::Odometry::ConstPtr r1;
  getData(bag,grid,scan,r0,r1);
  bag.close();
  FnF_mock fnf1;
  FnFData data;
  data.laserscan = *scan;
  data.pose0 = r0->pose.pose;
  data.pose1 = r1->pose.pose;
  fnf1.setData(data,*grid);
  geometry_msgs::PoseArray array = fnf1.standbeside();
  //need to get the point to the right of r1
  geometry_msgs::Point right; //point to the right taken from looking at sim
  right.x = -3.4;             //r1 coords = (-4.4,-4.4) -> -4.4 + 1 = -3.4
  right.y = -4.4;            //y remains the same
  
  EXPECT_NEAR(array.poses[0].position.x,right.x,0.2);
  EXPECT_NEAR(array.poses[0].position.y,right.y,0.2);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    //ros::init(argc, argv, "ass3_test");
    return RUN_ALL_TESTS();
}