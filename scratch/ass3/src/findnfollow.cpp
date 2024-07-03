#include "findnfollow.h"
#include "tf/transform_datatypes.h"
#include "std_msgs/Empty.h"
#include "utilities.h"

//double diameter = 0.46;

FnF::FnF() 
{
    datagot_ = false;
    following_ = false;
    insight_ = false;
    running = true;
    moving_ = false;
    searching_ = true;
    interrupt_ = false;
    diameter = 0.46;
    lost_ = false;
    //interrupt_ = false;
    
}

FnF::FnF(bool threads)
{
    datagot_ = false;
    following_ = false;
    insight_ = false;
    running = true;
    moving_ = false;
    searching_ = true;
    interrupt_ = false;
    diameter = 0.46;
    lost_ = false;
    start();
}

void FnF::start()
{
    threads_.push_back(std::thread(&FnF::search,this));
    threads_.push_back(std::thread(&FnF::following,this));
    threads_.push_back(std::thread(&FnF::checkLoS,this));
}

FnF::~FnF()
{
    running = false;
    //Join threads
    for(auto &t: threads_)
    {
        t.join();
    }
}

void FnF::following()
{
    FnFData data;
    geometry_msgs::PoseArray poses;
    geometry_msgs::Pose current;
    geometry_msgs::Pose current1;
    geometry_msgs::Point goal;
    double na;
    bool print = false;
    ros::Rate rate_limiter(1.0/1.0);
    std::unique_lock<std::mutex> lck(Followmtx_);
    Followcv_.wait(lck,[&](){return following_ == true;});
    lck.unlock();
    while(ros::ok())
    {
        
        
        mtx_.lock();
        data = data_;
        current = data_.pose0;
        current1 = data_.pose1;
        mtx_.unlock();
        geometry_msgs::Pose temp;
        temp.position = current1.position;
        goal = getpointaroundR1(current.position,temp.position);
        

        double yaw = atan2((temp.position.y-current.position.y),(temp.position.x-current.position.x));
        if(yaw<0){yaw = yaw + 2*M_PI;}
        YawtoQuat(yaw,temp);
        temp.position = goal;
        if(!lost_)
        {
        Pathpacket packet = {current,current1,temp};
        double dist_t_r1 = pow(pow((current1.position.y-current.position.y),2) + pow((current1.position.x - current.position.x),2),0.5);
        if(dist_t_r1>1.25)                      //if too close dont do anything
        {
            ROS_INFO_STREAM("FOLLOWING!");
            geometry_msgs::PoseArray array = getpath(packet);
            publishPose(array);
            ROS_INFO_STREAM("GOAL REACHED!");
            mtx_.lock();
            data = data_;
            current = data_.pose0;
            current1 = data_.pose1;
            mtx_.unlock();
            
            yaw = atan2((current1.position.y-current.position.y),(current1.position.x-current.position.x));
            if(yaw<0){yaw = yaw + 2*M_PI;}
            YawtoQuat(yaw,temp);

            temp.position = current.position;
            geometry_msgs::PoseArray tempa;
            tempa.poses = {temp};
            publishPose(tempa);
            print = false;
        }
        if(dist_t_r1<1.25)
        {
            if(!print)
            {
                ROS_WARN_STREAM("Too close! Remaining still. Can I help you?");
                print = true;
            }
            rate_limiter.sleep();
        }
        }
    }
}

