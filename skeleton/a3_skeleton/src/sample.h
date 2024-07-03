#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <mutex>

#include "ros/ros.h"
#include "tf/transform_datatypes.h" //To use getYaw function from the quaternion of orientation

//! All the messages for all projects are here
//! Use only those that you need, remove those you do not
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose2D.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseArray.h"

//! All the messages for all projects are here
//! Use only those that you need, remove those you do not
#include "project_setup/FaceGoal.h"
#include "project_setup/DetectParking.h"
#include "project_setup/RequestGoal.h"
#include "std_srvs/Trigger.h"

#include <atomic>

//! The class we have developed included here in our node
#include "grid_processing.h"


class Sample{

public:
  /*! @brief Sample constructor.
   *
   *  Will take the node handle and initialise the callbacks and internal variables
   */
    Sample(ros::NodeHandle nh);

  /*! @brief Sample destructor.
   *
   *  Will tear down the object
   */
    ~Sample();


  /*! @brief Example of a function for a service callback
   *  @note Change this to refelcet the service(s) you need to respond to
   *  @param req The requested goal.
   *  @param res The responce
   *
   *  @return bool - Will return true to indicate the request sucseeded
   */
    bool myExample(project_setup::FaceGoal::Request  &req,
             project_setup::FaceGoal::Response &res);


  /*! @brief Odometry Callback
   *
   *  @param nav_msgs::OdometryConstPtr - The odometry message
   *  @note This function and the declaration are ROS specific
   */
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

  /*! @brief LaserScan Callback
   *
   *  @param sensor_msgs::LaserScanConstPtr - The laserscan message
   *  @note This function and the declaration are ROS specific
   */
    void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);

  /*! @brief OccupancyGrid Callback
    *
    *  @param sensor_msgs::ImageConstPtr - The imageconst message
    *  @note This function and the declaration are ROS specific
    */
     void occupancyGridCallback(const nav_msgs::OccupancyGridPtr & msg);

  /*! @brief seperate thread.
   *
   *  The main processing thread that will run continously and utilise the data
   *  When data needs to be combined then running a thread seperate to callback will gurantee data is processed
   */
    void seperateThread(); 


private:
    /*! @brief Creates a MarkerArray by adding a single SPHERE marker to it, based on (pose)x,y position
     * of point supplied and color supplied, marker is in namespace "test",
     * @note We do not use pose for the markers, though you can pass a full pose, especially for arrows
     *
     *  @param geometry_msgs::Pose point - only the x,y position will be used
     *  @param std_msgs::ColorRGBA color - the color (including transparency) that will be used
     *  @param[in|out] id - in an array all markers should have a unique id, so we pass this by ref to be incremented
      */
    visualization_msgs::Marker produceMarkerSphere(geometry_msgs::Point point, std_msgs::ColorRGBA color,
                                                   int& id);


    ros::NodeHandle nh_;//! Node handle
    ros::Publisher viz_pub_;//! Visualisation Marker publisher

    ros::Subscriber sub1_,sub2_,sub3_; //! Subscribers
    ros::ServiceServer service_; //! Advertised Service


    struct PoseDataBuffer
    {
        geometry_msgs::Pose pose;
        std::mutex mtx;
    };

    struct OgMapBuffer
    {
        nav_msgs::OccupancyGrid grid;
        std::mutex mtx;
    };

    OgMapBuffer ogMapBuffer_;//! Container for occupancygrid data, so it's accessible elsewhere
    PoseDataBuffer poseDataBuffer_;//! Container for pose data, so it's accessible elsewhere
    std::atomic<bool> exampleBool_; //! Atomic bool can be used between threads, callback

};

