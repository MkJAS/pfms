#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <chrono>
#include <deque>
#include <mutex>
#include <random>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h" //To use getYaw function from the quaternion of orientation

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

/**
 * This node shows some connections and publishing images
 */


class RobotControl{

public:
  /*! @brief RobotControl constructor.
   *
   *  Will take the node handle and initialise the callbacks and internal variables
   */
    RobotControl(ros::NodeHandle nh);

  /*! @brief RobotControl destructor.
   *
   *  Will tear down the object
   */
    ~RobotControl();

    /*! @brief seperate thread.
     *
     *  The main processing thread that will run continously and utilise the data
     *  When data needs to be combined then running a thread seperate to callback will gurantee data is processed
     */
    void seperateThread();

private:
    /*! @brief LaserScan Callback
   *
   *  @param sensor_msgs::LaserScanConstPtr - The laserscan message
   *  @note This function and the declaration are ROS specific
   */
    void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);

  /*! @brief CmdVeocity Callback
    *
    *  @param geometry_msgs::TwistConstPtr - The command velocity message
    *  @note This function and the declaration are ROS specific
    */
    void cmdVelCallback(const geometry_msgs::TwistConstPtr & msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub1_;
    ros::Subscriber sub2_;
    ros::Publisher cmd_vel_pub_;//! Command velocity publisher

    struct CmdVelBuffer
    {
      //! Question: Given these elements come in two's (pose and time)
      //! Is there a better type of STL container rather than two seperate deques?
        geometry_msgs::Twist twist;
        std::mutex mtx;
    };
    CmdVelBuffer cmdVelBuffer_;//! Container for command velocity data

};

