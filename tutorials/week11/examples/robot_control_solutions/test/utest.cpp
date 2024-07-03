#include <gtest/gtest.h>
#include <climits>
#include <vector>

#include <ros/package.h> //This tool allows to identify the path of the package on your system
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/LaserScan.h>

#include "../src/mystuff.h"
#include "../src/circle.h"
#include "../src/data.h"

TEST(LoadedBag,CircleDetection){

    //! The below code tests circle detection
    //! On saved circle data

    //! Create an object of image processing as we will use the public function of that object to run tests against
    //ImageProcessing imageProcessing;

    //! Below command allows to find the folder belonging to a package
    std::string path = ros::package::getPath("random_walk");
    // Now we have the path, the images for our testing are stored in a subfolder /test/samples
    path += "/test/bag/";
    // The file is called smiley_og.png
    std::string file = path + "sample.bag";

    //! Manipulating rosbag, from: http://wiki.ros.org/rosbag/Code%20API
    rosbag::Bag bag;
    bag.open(file);  // BagMode is Read by default
    sensor_msgs::LaserScan::ConstPtr laserScan;

    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
      laserScan = m.instantiate<sensor_msgs::LaserScan>();
      if (laserScan != nullptr){
        // Now we have a laserScan so we can proceed
        break;
      }
    }

    bag.close();

    ASSERT_NE(laserScan, nullptr);//Check that we have a scan from the bag


    unsigned int highIntensityScans = 0;
    for (int elem = 0; elem < laserScan->ranges.size(); elem++) {
      if (laserScan->intensities.at(elem)>0){
        highIntensityScans++;
      }
    }
      std::cout << std::endl;

    ASSERT_GT(highIntensityScans,0);//Should have several High Intensity Readings

    double x[highIntensityScans];
    double y[highIntensityScans];

    unsigned int idx=0;
    for (int elem = 0; elem < laserScan->ranges.size(); elem++) {
      if (laserScan->intensities.at(elem)>0){
        highIntensityScans++;
        float range = laserScan->ranges.at(elem);
        x[idx] = range * cos(laserScan->angle_min+(elem*laserScan->angle_increment));
        y[idx] = range * sin(laserScan->angle_min+(elem*laserScan->angle_increment));
        std::cout << x[idx] << "," << y[idx] << "," ;
        idx++;
      }
    }

    Data data(highIntensityScans,x,y);
    Cirle cincleIni(3, 0, 2);
    double LambdaIni = 0.001;
    Circle cicle;


    int ret = CircleFitByChernovLesort (data, circleIni, LambdaIni, circle) ;



//[3.000 0.333 0.000 0.000])
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
