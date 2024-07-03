#ifndef DETECTION_H
#define DETECTION_H

#include <sstream>
#include <iostream>
#include <string>
#include <vector>

#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <atomic>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h" 

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "project_setup/RequestGoal.h"

#include "findnfollow.h"
#include "findnfollow_adv.h"
#include "utilities.h"



class Detection
{
public:
    Detection(ros::NodeHandle nh,int mode);
    ~Detection();

    void seperatethread();
    void sendLineofSightdata();


private:
    int mode_;
    ros::NodeHandle nh_;

    ros::Subscriber sub0_;
    ros::Subscriber sub1_;
    ros::Subscriber laser_sub_;
    ros::Subscriber ogmap_sub_;
    //ros::ServiceServer service_;

    //ros::Publisher pose_pub_;//! Command velocity publisher

    void odomCallback0(const nav_msgs::OdometryConstPtr& msg);
    void odomCallback1(const nav_msgs::OdometryConstPtr& msg);
    void laserCallback(const sensor_msgs::LaserScan& msg);
    void occupancyGridCallback(const nav_msgs::OccupancyGridPtr& msg);

    struct PoseDataBuffer
    {
        geometry_msgs::Pose pose;
        std::mutex mtx;
    };
    PoseDataBuffer poseData0_;//! Container for robot0 pose data
    PoseDataBuffer poseData1_;//! Container for robot1 pose data

    struct LaserScan
    {
        // std::vector<float> ranges;
        // std::vector<float> intens;
        sensor_msgs::LaserScan laser_scan;
        std::mutex mtx;
    };

    struct OgMapBuffer
    {
        nav_msgs::OccupancyGrid grid;
        std::mutex mtx;
    };

    OgMapBuffer ogMapBuffer_;

    FnF FnF_;    
    FnFadv FnFadv_;    

    LaserScan laserScan_;
    geometry_msgs::Pose pose0_;
    geometry_msgs::Pose pose1_;
    sensor_msgs::LaserScan laserscan_;
    nav_msgs::OccupancyGrid grid_;
    std::atomic<bool> LoS_data_got;
    std::condition_variable LoS_cv_; 


    //To shared across threads let's make
    std::atomic<bool> poseRequested_;/*!< The boolean to indicate if we have recieved a request */
    PoseDataBuffer goalBuffer_;
};

#endif //Detection_H