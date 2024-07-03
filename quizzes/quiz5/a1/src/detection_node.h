#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <mutex>

#include "ros/ros.h"

#include "sensor_msgs/LaserScan.h"

//! All the services .. DetectParking is part of project_setup, not a standard service
#include "project_setup/DetectParking.h"

#include "laserprocessing.h"


/**
 * This node shows some connections and publishing images
 */


class Detection{

public:
  /*! @brief Detection constructor.
   *
   *  Will take the node handle and initialise the callbacks and internal variables
   */
    Detection(ros::NodeHandle nh);

  /*! @brief Detection destructor.
   *
   *  Will tear down the object
   */
    ~Detection();


    /*! @brief Detect Cabinet service callback
     *
     *  @param req The requested goal.
     *  @param res The responce
     *
     *  @return bool - Will return true to indicate the request sucseeded
     */
      bool detectCabinet(project_setup::DetectParking::Request  &req,
               project_setup::DetectParking::Response &res);

private:

      /*! @brief LaserScan Callback
        *
        *  @param sensor_msgs::LaserScanConstPtr - The laserscan message
        *  @note This function and the declaration are ROS specific
        */
         void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);



private:
    ros::NodeHandle nh_;
    ros::Subscriber sub1_;
    ros::ServiceServer service_;
    int prev_high_intensity_ = 0;

    struct LaserData
    {
        sensor_msgs::LaserScan scan;
        std::mutex mtx;
    } laserData_;

};

