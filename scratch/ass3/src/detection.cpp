#include "detection.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

Detection::Detection(ros::NodeHandle nh,int mode)
    : nh_(nh)
{
    sub0_ = nh_.subscribe("/robot_0/odom", 1000, &Detection::odomCallback0,this);
    sub1_ = nh_.subscribe("/robot_1/odom", 1000, &Detection::odomCallback1,this);
    laser_sub_ = nh_.subscribe("/robot_0/base_scan", 1000, &Detection::laserCallback,this);
    ogmap_sub_ = nh_.subscribe("local_map/local_map", 1, &Detection::occupancyGridCallback,this);

    //pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("robot_0/path",1,false);
    
    //ros::Publisher idk = nh_.advertise<visualization_msgs::MarkerArray>("robot_0/following",3,false);
    mode_ = mode;
    if(mode != 2)
    {
        //FnFs_.push_back(new FnF);
        //FnFs_[0]->setHandle(nh_);
        FnF_.setHandle(nh_);
    }
    else 
    {
        //FnFs_.push_back(new FnFadv);
        //FnFs_[0]->setHandle(nh_);
        FnFadv_.setHandle(nh_);
    }
    

}

Detection::~Detection()
{

}


void Detection::odomCallback0(const nav_msgs::OdometryConstPtr& msg)
{
    std::unique_lock<std::mutex> lck (poseData0_.mtx);
    poseData0_.pose = msg->pose.pose;
}

void Detection::odomCallback1(const nav_msgs::OdometryConstPtr& msg)
{
    std::unique_lock<std::mutex> lck (poseData1_.mtx);
    poseData1_.pose = msg->pose.pose;
}

void Detection::laserCallback(const sensor_msgs::LaserScan& msg)
{
    std::unique_lock<std::mutex> lck (laserScan_.mtx);
    laserScan_.laser_scan = msg;
}

void Detection::occupancyGridCallback(const nav_msgs::OccupancyGridPtr& msg)
{

  std::unique_lock<std::mutex> lck (ogMapBuffer_.mtx);
  ogMapBuffer_.grid = *msg;

}


void Detection::seperatethread()
{
 
    geometry_msgs::PoseArray pub_pose;
    geometry_msgs::Point p;
    geometry_msgs::Quaternion q;
    geometry_msgs::Pose pq;
    nav_msgs::OccupancyGrid grid;
    std::vector<geometry_msgs::Pose> ps;
    int i = 0;

    poseRequested_ = true;
    //nav_msgs::OccupancyGrid grid;
    
    //! rate limiter runs code every 2 seconds
    ros::Rate rate_limiter(1.0/2.0);
    rate_limiter.sleep();
    while (ros::ok()) 
    {
        
        // if(i!=2){
        // i = 0;}
        
        if(poseRequested_)
        {
            //! Get the Pose message
            poseData0_.mtx.lock();
            pose0_ = poseData0_.pose;
            poseData0_.mtx.unlock();

            poseData1_.mtx.lock();
            pose1_ = poseData1_.pose;
            poseData1_.mtx.unlock();

            laserScan_.mtx.lock();
            laserscan_ = laserScan_.laser_scan;
            laserScan_.mtx.unlock();

            ogMapBuffer_.mtx.lock();
            grid_ = ogMapBuffer_.grid;
            ogMapBuffer_.mtx.unlock();

            sendLineofSightdata();
        }
        
    }
}

void Detection::sendLineofSightdata()
{
    nav_msgs::OccupancyGrid grid;
    FnFData pack;
    pack.pose0 = pose0_;
    pack.pose1 = pose1_;
    pack.laserscan = laserscan_;
    grid = grid_;
    if(mode_ != 2)
    {
        FnF_.setData(pack,grid);
    }
    else 
    {
        FnFadv_.setData(pack,grid);
    }
    
}