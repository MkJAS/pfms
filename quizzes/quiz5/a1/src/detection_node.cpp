#include "detection_node.h"

Detection::Detection(ros::NodeHandle nh)
    : nh_(nh)
{
    sub1_ = nh_.subscribe("/robot_0/base_scan", 10, &Detection::laserCallback,this);

    //Allowing an incoming service on /detect_cabinet
    service_ = nh_.advertiseService("/detect_cabinet", &Detection::detectCabinet,this);
}

Detection::~Detection()
{
    //asdjia = 4; 
}


void Detection::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
   /**
   * This callback store laser data to be used in service call
   */

    std::lock_guard<std::mutex> lck(laserData_.mtx);
    laserData_.scan = *msg;
}

bool Detection::detectCabinet(project_setup::DetectParking::Request  &req,
                                   project_setup::DetectParking::Response &res)
{
    /**
   * We here make a service call that uses the class.
   */
    sensor_msgs::LaserScan scan;
    {
        std::lock_guard<std::mutex> lck(laserData_.mtx);
        scan = laserData_.scan;
    }
    

    LaserProcessing laserprocessing(scan);
    geometry_msgs::Pose pose = laserprocessing.detectPoseHighIntensity();

    ROS_INFO_STREAM("Cabinet corner location: " << pose.position.x << ", "
                    << pose.position.y << ", orientation: " << tf::getYaw(pose.orientation));
    res.pose.poses.push_back(pose);
    return true;
}

