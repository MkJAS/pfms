#include <gtest/gtest.h>
#include <climits>
#include <vector>

#include <ros/package.h> //This tool allows to identify the path of the package on your system
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h" //To use getYaw function from the quaternion of orientation

#include "../src/ogmapprocessing.h"


TEST(OgMapProcessing,CountUnknownCells){

}


TEST(OgMapProcessing,IsLocationFree){

  //! The below code tests the laserprocessing class
  //! The data has been saved in a bag, that is opened and used.
  //! Unforttunately as we need to use a ConstPtr below, we can't make this
  //! a helper function

  //! Below command allows to find the folder belonging to a package
  std::string path = ros::package::getPath("quiz5_a1");
  // Now we have the path, the images for our testing are stored in a subfolder /test/samples
  path += "/test/bag/";
  std::string file = path + "sample.bag";

  //! Manipulating rosbag, from: http://wiki.ros.org/rosbag/Code%20API
  rosbag::Bag bag;
  bag.open(file);  // BagMode is Read by default
  nav_msgs::OccupancyGrid::ConstPtr ogMap = nullptr;
  nav_msgs::Odometry::ConstPtr odom = nullptr;

  //! The bag has all the messages, so we go through all of them to find the mesages we need
  for(rosbag::MessageInstance const m: rosbag::View(bag))
  {
    //! We will go through the bag and extract the laser scan and odometry
    //! We have to try to instatitate each message type

    if(m.getTopic() == "/local_map/local_map"){
      if( ogMap == nullptr){
        ogMap = m.instantiate<nav_msgs::OccupancyGrid>();
      }
    }
    if(m.getTopic() == "/robot_0/odom"){
      if( odom == nullptr){
        odom = m.instantiate<nav_msgs::Odometry>();
      }
    }
    if ((ogMap != nullptr) && (odom != nullptr)){
      //! Now we have a laserScan and odometry so we can proceed
      //! We could also check here if we have High Intensity readings before abandoning the loop
      break;
    }
  }
  bag.close();


  ASSERT_NE(ogMap, nullptr);//Check that we have a laser scan from the bag
  ASSERT_NE(odom, nullptr);//Check that we have a laser scan from the bag

  ////////////////////////////////////////////
  // Our code is tested below


  //! Create an object of DetectCabinet class as we will use the public function of that object to run tests against
  OgmapProcessing ogmapProcessing(*ogMap);
  geometry_msgs::Point goal;
  goal.x=3.5;goal.y=3.5;
  EXPECT_TRUE(ogmapProcessing.isLocationFree(goal));
  goal.x=3.6;goal.y=4.6;
  EXPECT_FALSE(ogmapProcessing.isLocationFree(goal));

}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
