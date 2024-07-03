#ifndef LINEOFSIGHT_H
#define LINEOFSIGHT_H

//#include "detection.h"
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

#include "structs.h"



class LoS
{
public:
    LoS();
    ~LoS();
    void setData(LoSData data);
    void setHandle(ros::NodeHandle nh);

private:
    LoSData data_;
    bool datagot;
    void search();
    bool searchData(double& bearing_);
    std::atomic<bool> insight_;

    bool running;
    std::vector<std::thread> threads_;
    
    ros::Publisher pose_pub_;
    ros::NodeHandle nh_;

    std::condition_variable cv_;
    std::mutex mtx_;

};

#endif