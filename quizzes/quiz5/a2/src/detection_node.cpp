
#include "detection_node.h"
#include "ogmapprocessing.h"


/**
 * This node shows some connections and publishing images
 */


Detection::Detection(ros::NodeHandle nh)
    : nh_(nh)
{
    //Subscribing to odometry (Do we need to change thge topic???)
    sub1_ = nh_.subscribe("/robot_0/odom", 1000, &Detection::odomCallback,this);

    //Subscribing to occupnacy grid
    sub2_ = nh_.subscribe("local_map/local_map", 1, &Detection::occupancyGridCallback,this);

    //Allowing an incoming service on /check_goal
    service_ = nh_.advertiseService("check_goal", &Detection::requestGoal,this);
}

Detection::~Detection()
{

}


bool Detection::requestGoal(project_setup::RequestGoal::Request  &req,
             project_setup::RequestGoal::Response &res)
{
  //The incoming request is a Global coordinate.

  poseRequested_=false;
  geometry_msgs::Pose reqPose;
  //When an incoming call arrives, we can respond to it here
  if (req.pose.poses.size()>0){
    for (auto pose : req.pose.poses){
      ROS_INFO_STREAM("requested: [x,y]=[" << pose.position.x << "," << pose.position.y);
    }
    ROS_INFO_STREAM("Current support is for only the fisrt positon requested");
    reqPose= req.pose.poses.at(0);
    goalBuffer_.mtx.lock();
    goalBuffer_.pose = reqPose;
    goalBuffer_.mtx.unlock();
    poseRequested_=true;
  }
  else {
    ROS_INFO_STREAM("requested an empty list of poses");
  }

  if(poseRequested_){
    poseDataBuffer_.mtx.lock();
    geometry_msgs::Pose pose=poseDataBuffer_.pose;
    poseDataBuffer_.mtx.unlock();

    //! Get the OgMap message
    ogMapBuffer_.mtx.lock();
    nav_msgs::OccupancyGrid grid= ogMapBuffer_.grid;
    ogMapBuffer_.mtx.unlock();

    OgmapProcessing ogmapProcessing(grid);
    bool OK = ogmapProcessing.isLocationFree(reqPose.position);
  }

  return true;
}

void Detection::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    //! REMEBER: on command line you can view entier msg as
    //! rosmsg show nav_msgs/Odometry
    std::unique_lock<std::mutex> lck (poseDataBuffer_.mtx);
    poseDataBuffer_.pose = msg->pose.pose;
}



void Detection::occupancyGridCallback(const nav_msgs::OccupancyGridPtr& msg)
{

  std::unique_lock<std::mutex> lck (ogMapBuffer_.mtx);
  ogMapBuffer_.grid = *msg;

}


void Detection::seperateThread() {
   /**
    * The below loop runs until ros is shutdown, to ensure this thread does not remain
    * a zombie thread
    *
    */

   geometry_msgs::Pose pose;
   geometry_msgs::Pose reqPose;
   nav_msgs::OccupancyGrid grid;
   int numCells=0;

    //! rate limiter runs code every 5 seconds
    ros::Rate rate_limiter(1.0/5.0);

    while (ros::ok()) {


      if(poseRequested_){
        //! Get the Pose message
        poseDataBuffer_.mtx.lock();
        pose=poseDataBuffer_.pose;
        poseDataBuffer_.mtx.unlock();

        goalBuffer_.mtx.lock();
        reqPose=goalBuffer_.pose;
        goalBuffer_.mtx.unlock();

        //! Get the OgMap message
        ogMapBuffer_.mtx.lock();
        grid= ogMapBuffer_.grid;
        ogMapBuffer_.mtx.unlock();


        OgmapProcessing ogmapProcessing(grid);
        bool OK = ogmapProcessing.isLocationFree(reqPose.position);

        if(OK){
          ROS_INFO_STREAM("Cell is free");
        }
        else {
          ROS_INFO_STREAM("Cell is NOT free");
        }
    }

        rate_limiter.sleep();
    }
}

