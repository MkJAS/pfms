#include "lineofsight.h"
#include "tf/transform_datatypes.h"



LoS::LoS()
{
    datagot = false;
    insight_ = false;
    running = true;
    threads_.push_back(std::thread(&LoS::search,this));

}

LoS::~LoS()
{
    running = false;
    //Join threads
    for(auto &t: threads_)
    {
        t.join();
    }

}



void LoS::setData(LoSData data)
{
    std::unique_lock<std::mutex> lck(mtx_);
    data_ = data;
    lck.unlock();
    datagot = true;
    cv_.notify_all();
}

void LoS::search()
{
    while(running)
    {
        geometry_msgs::Pose pose;
          
        geometry_msgs::PoseArray pub_pose;
        double yaw = 0;
        //ros::Rate rate_limiter(1.0/2);
        std::unique_lock<std::mutex> lck(mtx_);
        cv_.wait(lck,[&](){return datagot == true;});
        lck.unlock();
        searchData(yaw);
        geometry_msgs::Pose p = data_.pose0;
        while(!insight_)
        {
          
        //   yaw = tf::getYaw(p.orientation) + M_PI;
        //   if(yaw>2*M_PI)
        //   {
        //     yaw = tf::getYaw(p.orientation) + M_PI;
        //   }
          ROS_INFO_STREAM("yaw: "<<yaw);

          //yaw = -1*yaw;
          //geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(yaw);
          tf::Quaternion quat = tf::createQuaternionFromYaw(yaw);
          pose.orientation.w = quat.getW();
          pose.orientation.x = quat.getX();
          pose.orientation.y = quat.getY();
          pose.orientation.z = quat.getZ();
          pose.position = p.position;
          ROS_INFO_STREAM("yaw: "<<yaw);
          //pose.orientation = quat;
          pub_pose.poses = {pose};
          //geometry_msgs::Pose pub_pose [1] = {pose};
          pose_pub_.publish(pub_pose);
          mtx_.lock();
          double o = data_.pose0.orientation.w;
          mtx_.unlock();
          while(o+0.005 < pose.orientation.w && o-0.005 > pose.orientation.w)
          {
              mtx_.lock();
              o = data_.pose0.orientation.w;
              mtx_.unlock();
          }
          //rate_limiter.sleep();
          searchData(yaw);
        }
    }

}

bool LoS::searchData(double& bearing_) 
{
    LoSData data = data_;
    geometry_msgs::Point xy1;
    geometry_msgs::Point xy0;
    geometry_msgs::Point r1_local;

    double orient0;
    double angle;
    xy1.x = data.pose1.position.x;
    xy1.y = data.pose1.position.y;
    orient0 = tf::getYaw(data.pose0.orientation);
    if(orient0<0){orient0 += 2*M_PI;}
    //ROS_INFO_STREAM("Orient: "<<orient0);
    xy0.x = data.pose0.position.x;
    xy0.y = data.pose0.position.y;

    r1_local.x = (xy1.x - xy0.x)*cos(orient0) + (xy1.y - xy0.y)*sin(orient0);
    r1_local.y = -(xy1.x - xy0.x)*sin(orient0) + (xy1.y - xy0.y)*cos(orient0);

    double dist = pow((pow((r1_local.x),2) + pow((r1_local.y),2)),0.5);
    double bearing = atan2(r1_local.y,r1_local.x);
    bearing_ = atan2((xy1.y-xy0.y),(xy1.x-xy0.x));
    if(bearing_<0){bearing_ += 2*M_PI;}
    if(r1_local.x<0)
    {
        insight_ = false;
        return false;
    }
    if(bearing<0)
    {
        angle = M_PI/2 - (bearing*-1);
    }
    if(bearing>0)
    {
        angle = M_PI/2 + bearing;
    }
    int rng_num = round(angle/data.laserscan.angle_increment);
    //int val = data.laserscan.ranges[rng_num];
    if(data.laserscan.ranges[rng_num]<dist+0.2 && data.laserscan.ranges[rng_num]>dist-0.2)
    {
        if(data.laserscan.intensities[rng_num]==1 || data.laserscan.intensities[rng_num]==1 || data.laserscan.intensities[rng_num]==1)
        {
            insight_ = true;
            return true;
        }
        else
        {
            insight_ = false;
            return false;
        }
    }
    else
    {
        insight_ = false;
        return false;
    }
}


void LoS::setHandle(ros::NodeHandle nh)
{
    nh_ = nh;
    pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("robot_0/path",1,false);
}