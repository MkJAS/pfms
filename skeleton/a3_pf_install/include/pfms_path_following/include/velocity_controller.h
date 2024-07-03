/**
 * @file    velocity_control.h
 * @ingroup Control
 * @class   VelocityController
 * @brief   Controls velocity to pursue goal poses 
 * @author  Ajal Singh
 * @version 1.0
 * @date    June 2020
 */

#ifndef VELOCITY_CONTROLLER_H
#define VELOCITY_CONTROLLER_H

//_____________________________________________________________________________ Includes
#include "ros/ros.h"

// Messages
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "visualization_msgs/MarkerArray.h"

#include <thread>
#include <deque>
#include <mutex>
#include <atomic>
#include <cmath>

#include "controller_helper.h"
#include "geometry.h"
#include "pure_pursuit.h"

//_____________________________________________________________________________ Class Definition
class VelocityController : public VelocityControlHelper
{
//_____________________________________________________________________________ Class Public Members
public:
    /**
     * @brief Construct a new Velocity Controller object
     * 
     * @param nh 
     */
    VelocityController(ros::NodeHandle nh);

    /**
     * @brief Destroy the Velocity Controller object
     * 
     */
    ~VelocityController();

    /**
     * @brief Control robot to reach goal poses while avoiding obstacles
     *
     */
    void pursueGoal();

//_____________________________________________________________________________ Class Private Members
private:

    /**
     * @brief Retrieves messages from /move_base_simple/goal topic
     *
     * @param msg
     */
    void poseCallback(const geometry_msgs::PoseStampedPtr &msg);

    /**
     * @brief Retrieves messages from /robot_0/path topic
     *
     * @param msg
     */
    void pathCallback(const geometry_msgs::PoseArrayPtr &msg);

    /**
     * @brief Retrieves messages from /robot_0/odom topic
     *
     * @param msg
     */
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

    /**
     * @brief Retrieves messages from /robot_0/base_scan topic
     *
     * @param msg
     */
    void laserCallback(const sensor_msgs::LaserScanPtr& msg);


    void produceMarkerArray(geometry_msgs::Pose pose, std_msgs::ColorRGBA color);


    inline std_msgs::ColorRGBA getColor(float r, float g, float b){
      std_msgs::ColorRGBA color;
      color.a=0.5;color.r=r;color.g=g;color.b=b;
      return color;
    }

    //_____________________________________________________________________________ Private Member Variables
    ros::NodeHandle nh_;                                            //!< nodehandle

    ros::Subscriber path_sub_;                      //!< Subscriber to /robot_0/path topic
    ros::Subscriber odom_sub_;                      //!< Subscriber to /robot_0/odom topic
    ros::Subscriber base_scan_sub_;                 //!< Subscriber to /robot_0/base_scan topic
    ros::Subscriber pose_sub_;                      //!< Subscriber to /move_base_simple/goal topic

    ros::Publisher vel_pub_;                        //!< Publisher to /robot_0/cmd_vel topic
//    ros::Publisher raw_path_pub_;                   //!< Publisher to /raw_path topic
//    ros::Publisher path_pub_;                       //!< Publisher to /robot_0/path topic
//    ros::Publisher compl_path_pub_;                 //!< Publisher to /robot_0/completed_path topic
//    ros::Publisher current_goal_pub_;               //!< Publisher to /robot_0/current_goal topic

    ros::Publisher viz_pub_;                        //!< Publishing for visualisation

    Geometry geometry_;                             //!< Instance of Geometry class
    PurePursuit pp_controller_;                     //!< Instance of PurePursuit class    

    /**
     * @brief Buffer to hold threadsafe path
     * 
     */
    struct PoseArrayBuffer
    {
        std::mutex mtx;                             //!< mutex to lock data
        std::atomic<bool> received;                 //!< bool to indicate data to be sent
        geometry_msgs::PoseArray path;              //!< series of poses as PoseArray
    };
    PoseArrayBuffer checkpoints_buffer_;            //!< threadsafe buffer used to store path
    PoseArrayBuffer completed_checkpoints_buffer_;  //!< threadsafe buffer used to store completed path

    /**
     * @brief Buffer to hold threadsafe goal pose
     * 
     */
    struct Goal
    {
        geometry_msgs::PoseStamped goalPose;        //!< Current goal pose
        geometry_msgs::Pose vehPose;                //!< Current vehicle pose
        std::atomic<bool> hasGoal;                  //!< Flag to check if current goal exists
        std::atomic<bool> pointReached;             //!< Flag to check if goal point reached
        std::atomic<bool> poseCompleted;            //!< Flag to check if goal psoe completed
        std::mutex mtx;                             //!< Mutex to lock data
    };
    Goal goal_buffer_;                              //!< threadsafe buffer used to store goal pose

    /**
     * @brief Buffer to hold threadsafe obstacle params
     * 
     */
    struct ObstacleBuffer{
        bool obstacle;                              //!< True when obstacle detected
        bool messageDisplayed;                      //!< Flag used to inform user of obstacle 
    };
    ObstacleBuffer obstacle_detected_;              //!< threadsafe buffer used to store obstacle params
   
   /**
    * @brief Buffer to hold threadsafe velocity and param
    * 
    */
   struct Velocity{
       bool purePursuit;                            //!< Pure pursuit param
       geometry_msgs::Twist twistMsg;               //!< Stores velocity values
   };
   Velocity velocity_;                              //!< threadsafe buffer used to store velocity

   visualization_msgs::MarkerArray markerArray_;     //!< markerArray published


};

#endif //VELOCITY_CONTROLLER_H
