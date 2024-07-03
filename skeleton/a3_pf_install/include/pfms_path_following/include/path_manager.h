/**
 * @file    path_manager.h
 * @ingroup PathManager
 * @class   PathManager
 * @brief   Handles all events path related
 * @details Converts 2D Nav Goals from rviz into a pose array to be used in other nodes.
 *          After each goal selection, the user has 5 seconds to select another before time-out.
 *          After the 5 second timeout, the PoseArry is published to /raw_path.
 * 
 *          This class also checks whether the current goal violates free space. 
 *          If yes, it is abandonded and we move to the next goal and check. 
 * @author  Ajal Singh
 * @version 1.0
 * @date    June 2020
 */

#ifndef CHECKPOINT_SUPPLIER_H
#define CHECKPOINT_SUPPLIER_H

//_____________________________________________________________________________ Includes

#include <sstream>
#include <thread>
#include <chrono>
#include <deque>
#include <mutex>
#include <random>
#include <atomic>

#include "ros/ros.h"

//! All the messages we need are here
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

//ROS-OpenCV Tools for Image Manipulation
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "ref_frame_conversion.h"
#include "image_processing.h"

//_____________________________________________________________________________ Global Variable

namespace enc = sensor_msgs::image_encodings;

//_____________________________________________________________________________ Class Definition
class PathManager{
//_____________________________________________________________________________ Class Public Members
public:
    /**
     * @brief Construct a new Path Manager object
     * 
     * @param nh rose nodehandle
     */
    PathManager(ros::NodeHandle nh);

    /**
     * @brief Destroy the Path Manager object
     * 
     */
    ~PathManager();

    /**
     * @brief Retrieves messages from /move_base_simple/goal topic
     * 
     * @param msg 
     */
    void poseCallback(const geometry_msgs::PoseStampedPtr &msg);

    /**
     * @brief Retrieves messages from /robot_0/odom topic
     * 
     * @param msg 
     */
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

    /**
     * @brief Retrieves messages from /map_image/full topic
     * 
     * @param msg 
     */
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    /**
     * @brief Retrieves messages from /raw_path topic
     * 
     * @param msg 
     */
    void rawPathCallback(const geometry_msgs::PoseArrayPtr &msg);

    /**
     * @brief Publishes PoseArry to /raw_path topic
     * 
     */
    void publishRawPath();

//_____________________________________________________________________________ Class Private Members
private:
//_____________________________________________________________________________ Private Member Functions
    bool checkGoalReachable(const geometry_msgs::Pose goal);

//_____________________________________________________________________________ Private Member Variables
    ros::NodeHandle nh_;                    //!< Nodehandle

    ros::Subscriber pose_sub_;              //!< Subscriber to /move_base_simple/goal topic
    ros::Subscriber raw_path_sub_;          //!< Subscriber to /raw_path topic
    ros::Subscriber odom_sub_;              //!< Subscriber to robot_0/odom topic
    image_transport::Subscriber map_sub_;   //!< Subscriber to /map_image/full topic
    cv_bridge::CvImagePtr cvPtr_;           //!< Pointer to convert ROS images to openCV images
    
    ros::Publisher raw_path_pub_;           //!< Publisher to /raw_path topic
    ros::Publisher path_pub_;               //!< Publisher to /robot_0/path topic

    RefFrameConversion refframeconversion_; //!< Instance of RefFrameConversion
    ImageProcessing imageprocessing_;       //!< Instance of ImageProcessing


    /**
     * @brief Buffer to hold threadsafe path
     * 
     */
    struct PoseArrayBuffer{
    std::mutex mtx;                         //!< mutex to lock data
        std::atomic<bool> send;             //!< bool to indicate array data to be sent, 
        geometry_msgs::PoseArray path;      //!< series of poses as PoseArray
        geometry_msgs::Pose vehPose;        //!< Current vehicle pose
        cv::Mat image;                      //!< opencv rgb OGMap 
    };
    PoseArrayBuffer poses_buffer_;          //!< threadsafe buffer used to stor incoming poses
    PoseArrayBuffer verified_path_;         //!< threadsafe buffer stores poses to be verified

    ros::Time start_time_;                  //!< start time
    ros::Duration duration_;                //!< duration since start time (seconds)
};
#endif // CHECKPOINT_SUPPLIER_H
