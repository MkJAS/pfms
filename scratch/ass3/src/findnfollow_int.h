#ifndef FINDNFOLLOWINT_H
#define FINDNFOLLOWINT_H

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
#include "visualization_msgs/MarkerArray.h"

#include "utilities.h"
#include "grid_processing.h"

/*!
 *  \brief     Find n Follow interface class
 *  \details   Contains the inherited functions for both basic and advanced mode classes.
 *  \author    Joseph Seklawy
 */


class FnFint
{
public:
    FnFint(){};
    ~FnFint(){};
    /**
     * @brief Collect the topic data and send it through to the FnF class
     * 
     * @param data a FnFData struct
     * @param grid 
     */
    void setData(FnFData data, nav_msgs::OccupancyGrid grid);

    /**
     * @brief Called from outside this class by the @ref TopicHandler and passes the ROS handle to the FnF class set in the main
     * 
     * @param nh  the node handle
     */
    void setHandle(ros::NodeHandle nh);

    /**
     * @brief Checks whether robot_1 is in line of sight of robot_0 by cross referencing the distance and angle between the 2 robots with the laser reading angles and ranges
     * 
     * @return true 
     * @return false 
     * @sa search
     */
    bool IsinLoS();

    /**
     * @brief Calculates wether a certain point is reachable by robot_0 and if it is not, it finds an intermidiary point which will then allow robot_0 to reach robot_1
     * 
     * @param pack struct containing robot_0 and robot_1 pose as well as the goal point around robot_1
     * @return geometry_msgs::PoseArray 
     * @sa goaround
     */
    geometry_msgs::PoseArray getpath(Pathpacket pack);

protected:
    /**
     * @brief Starts the required mode threads
     * 
     */
    virtual void start() = 0;
    /**
     * @brief Functions which computes poses for robot_0 to follow robot_1
     * @sa getpath
     * @sa goaround
     */
    virtual void following() = 0;
    /**
     * @brief Searches for robot_1 upon start up
     * @sa IsinLoS
     */
    void search();
    
    /**
     * @brief Part of the getpath function. If a point cannot be reached this functions works to find a point that can be reached
     * This function is mostly used when robot_1 turns a corner and robot_0 is unable to follow due to collision with the corner
     * Essentially this functions determines what kind of hallway robot_1 turned into, horizontal/vertical/angled and then determines a point within 
     * the hallway that robot_0 can reach.
     * Most of this assessment is done through various functions within the @ref utilities.h header
     *
     * 
     * @param pack 
     * @param corner1 
     * @param corner2 
     * @return geometry_msgs::PoseArray 
     * @sa getpath
     */
    geometry_msgs::PoseArray goaround(Pathpacket pack,geometry_msgs::Point corner1,geometry_msgs::Point corner2);
    /**
     * @brief Handles publishing poses and waiting until robot_0 has reached said pose before publishing a new pose
     * 
     * @param pose 
     */
    void publishPose(geometry_msgs::PoseArray pose);
    /**
     * @brief Handles displaying pose arrows within RVIZ
     * 
     * @param point 
     * @param color 
     * @param id 
     * @return visualization_msgs::Marker 
     */
    visualization_msgs::Marker produceMarker(geometry_msgs::Pose point, std_msgs::ColorRGBA color,int& id);

    void checkLoS();
    
    //!Struct which holds pose of robot_0,robot_1 as well as robot_0 base scan @sa utilities.h
    FnFData data_;
    nav_msgs::OccupancyGrid Grid_;
    geometry_msgs::PoseArray R1poses_;
    //! Bool set when robot_1 is first in sight    
    std::atomic<bool> insight_;
    std::atomic<bool> moving_;
          

    bool running;
    std::vector<std::thread> threads_;
    
    

    std::condition_variable cv_;
    std::condition_variable Followcv_;
    std::atomic<bool> datagot_;
    std::atomic<bool> following_;
    std::atomic<bool> searching_;
    std::atomic<bool> interrupt_;
    std::atomic<bool> lost_;

    std::mutex mtx_;
    std::mutex Followmtx_;
    std::mutex R1mtx_;
    //!diameter of robot_0
    double diameter; 

    //! Publisher to robot_0/path topic
    ros::Publisher pose_pub_;
    //! Publisher to robot_0/following topic
    ros::Publisher viz_pub_;
    ros::NodeHandle nh_;
};

#endif