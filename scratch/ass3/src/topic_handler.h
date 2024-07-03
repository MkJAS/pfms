#ifndef TOPICHANDLER_H
#define TOPICHANDLER_H

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



class TopicHandler
{
public:
    TopicHandler(ros::NodeHandle nh,int mode);
    ~TopicHandler();

    /**
     * @brief Thread that obtains topic messages and data
     * 
     */
    void seperatethread();

    /**
     * @brief Function which takes obtained set of robot odoms, robot0 laserscan and og map
     * and sends it over to relevant FnF class
     * 
     */
    void sendLineofSightdata();


private:
    /**
     * @brief Robot_0 odom callback
     * 
     * @param msg 
     */
    void odomCallback0(const nav_msgs::OdometryConstPtr& msg); 
    /**
     * @brief Robot_1 odom callback
     * 
     * @param msg 
     */
    void odomCallback1(const nav_msgs::OdometryConstPtr& msg);
    /**
     * @brief Robot_0 base_scan callback
     * 
     * @param msg 
     */
    void laserCallback(const sensor_msgs::LaserScan& msg);
    /**
     * @brief Occupancy grid callback
     * 
     * @param msg 
     */
    void occupancyGridCallback(const nav_msgs::OccupancyGridPtr& msg);

    
    struct PoseDataBuffer
    {
        geometry_msgs::Pose pose;
        std::mutex mtx;
    };
    PoseDataBuffer poseData0_;  // Container for robot0 pose data
    PoseDataBuffer poseData1_;  // Container for robot1 pose data

    struct LaserScan
    {
        sensor_msgs::LaserScan laser_scan;
        std::mutex mtx;
    };
    LaserScan laserScan_;


    struct OgMapBuffer
    {
        nav_msgs::OccupancyGrid grid;
        std::mutex mtx;
    };

    OgMapBuffer ogMapBuffer_;

    std::vector<FnFint*> FnFmode_; //vector to hold selected FnF class based on mode
       
    int mode_;
    ros::NodeHandle nh_;           

    ros::Subscriber sub0_;
    ros::Subscriber sub1_;
    ros::Subscriber laser_sub_;
    ros::Subscriber ogmap_sub_;

    geometry_msgs::Pose pose0_;
    geometry_msgs::Pose pose1_;
    sensor_msgs::LaserScan laserscan_;
    nav_msgs::OccupancyGrid grid_;
    std::atomic<bool> LoS_data_got;
    std::condition_variable LoS_cv_; 

    std::atomic<bool> poseRequested_;
    PoseDataBuffer goalBuffer_;
};

#endif //Detection_H