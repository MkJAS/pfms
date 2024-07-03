#include "findnfollow_adv.h"



FnFadv::FnFadv()
{
    datagot_ = false;
    following_ = false;
    insight_ = false;
    running = true;
    moving_ = false;
    beside_ = false;
    behind_ =  false;
    searching_ = true;
    interrupt_ = false;
    lost_ = false;
    diameter = 0.46;

}

FnFadv::FnFadv(bool threads)
{
    datagot_ = false;
    following_ = false;
    insight_ = false;
    running = true;
    moving_ = false;
    beside_ = false;
    behind_ =  false;
    searching_ = true;
    interrupt_ = false;
    lost_ = false;
    diameter = 0.46;
    start();

}

void FnFadv::start()
{
    
    threads_.push_back(std::thread(&FnFadv::search,this));
    threads_.push_back(std::thread(&FnFadv::following,this));
    threads_.push_back(std::thread(&FnFadv::getRobot1poses,this));
    threads_.push_back(std::thread(&FnFadv::distancecheck,this));
    threads_.push_back(std::thread(&FnFadv::checkLoS,this));
}

FnFadv::~FnFadv()
{
    running = false;
    //Join threads
    for(auto &t: threads_)
    {
        t.join();
    }
}

void FnFadv::following()
{
    ros::Duration(2.0).sleep();
    FnFData data;
    geometry_msgs::PoseArray poses;
    geometry_msgs::Pose current;
    geometry_msgs::Pose current1;
    geometry_msgs::Point goal;
    nav_msgs::OccupancyGrid grid;
    ros::Rate rate_limiter(1.0/2.0);
    //ros::Duration(2.0).sleep();
    std::unique_lock<std::mutex> lck(Followmtx_);
    Followcv_.wait(lck,[&](){return following_ == true;});
    lck.unlock();
    while(ros::ok())
    {
        geometry_msgs::Pose temp;
        geometry_msgs::Point goal_local;
        geometry_msgs::Point zero;
        
        
       
        
        mtx_.lock();
        data = data_;
        current = data_.pose0;
        current1 = data_.pose1;
        grid = Grid_;
        mtx_.unlock();
        if(!interrupt_ && !lost_)
        {
            temp.position = current1.position;
            goal = getpointbehindR1(current1);
            geometry_msgs::Point p;
            p.x = -0.3;
            p.y = 0;
            p = localtoglobal(current1,p); //find point just behind r1 to avoid false negatives. I.e. the case where its reported that the space is occupied due to the presence of r1 itself
            
            zero.x = p.x - current.position.x; //position of r1 in ogmap
            zero.y = p.y - current.position.y; 
            goal_local.x = goal.x - current.position.x;   //position behind r1 in ogmap
            goal_local.y = goal.y - current.position.y;

            GridProcessing gridproc(grid);
            bool reachable = gridproc.checkConnectivity(zero,goal_local); //check if the space between r1 and the point behind it is unobstructed
            double dist_t_r1 = pow(pow((current1.position.y-current.position.y),2) + pow((current1.position.x - current.position.x),2),0.5);
            //ROS_INFO_STREAM("Dist: "<<dist_t_r1);
            if(dist_t_r1>1.0)                     //if too close dont do anything
            {
        
                if(!reachable)
                {
                    ROS_WARN_STREAM("Point behind unreachable. Moving to closest available point");
                    goal = getpointaroundR1(current.position,current1.position);
                }      

                double yaw = atan2((temp.position.y-current.position.y),(temp.position.x-current.position.x)); //get angle between goal and current pos to supply an orientation
                if(yaw<0){yaw = yaw + 2*M_PI;}
                YawtoQuat(yaw,temp);
                temp.position = goal;

                Pathpacket packet = {current,current1,temp};
                ROS_INFO_STREAM("FOLLOWING!");
                geometry_msgs::PoseArray array = getpath(packet); //check point is reachable and find alternative if not
                publishPose(array);
                ROS_INFO_STREAM("GOAL REACHED!");
                mtx_.lock();
                data = data_;
                current = data_.pose0;  //get the new poses after reaching goal
                current1 = data_.pose1;
                mtx_.unlock();
            
                yaw = atan2((current1.position.y-current.position.y),(current1.position.x-current.position.x));
                if(yaw<0){yaw = yaw + 2*M_PI;}
                YawtoQuat(yaw,temp);    //get pose such that robot_0 will be facing robot_1 and publish it

                temp.position = current.position;
                geometry_msgs::PoseArray tempa;
                tempa.poses = {temp};
                publishPose(tempa);
            }
            if(dist_t_r1<1.0)
            {
                ROS_WARN_STREAM("Too close! Remaining still. Can I help you?");
                rate_limiter.sleep();
            }
        }
    }
}


void FnFadv::getRobot1poses()
{
    
    FnFData data;
    ros::Rate limiter(1/1.0); //1Hz
    geometry_msgs::Pose temp;
    geometry_msgs::PoseArray array;
    int count = 0;
    std::unique_lock<std::mutex> lck(Followmtx_);
    Followcv_.wait(lck,[&](){return following_ == true;});
    lck.unlock();
    ros::Duration(2.0).sleep();
    while(ros::ok())
    {
        
        mtx_.lock();
        data = data_;
        mtx_.unlock();
        geometry_msgs::Pose pose =  data.pose1;        
        if(temp.position.x == pose.position.x && temp.position.y == pose.position.y)
        {
            count++;
        }
        if(count >= 10 && !beside_) //if robot_1 has been still for 10 cycles, i.e. 10 seconds, move robot_0 to stand beside
        {
            interrupt_ = true;
            array.poses.clear(); //publish empty pose to abandon current poses
            publishPose(array);
            //following_ = false;
            ROS_INFO_STREAM("Moving to side.");
            array = standbeside();
            if(array.poses.size() == 1)
            {
                std_msgs::ColorRGBA color;
                color.a=1.0;//a is alpha - transparency 0.5 is 50%;
                color.r=0;
                color.g=1.0; 
                color.b=0;   

                int marker_counter=0;
                //Let's also publish the markerArray, we create a few markers
                visualization_msgs::MarkerArray marker_array;
                marker_array.markers.push_back(produceMarker(array.poses[0],color,marker_counter));
                viz_pub_.publish(marker_array);
                publishPose(array);

            }
            if(array.poses.size() == 2)
            {
                geometry_msgs::PoseArray temparray;
                temparray.poses.push_back(array.poses[0]);
                std_msgs::ColorRGBA color;
                color.a=1.0;//a is alpha - transparency 0.5 is 50%;
                color.r=0;
                color.g=0; 
                color.b=1.0;    //it will be blue - for interim pose

                int marker_counter=0;
                //Let's also publish the markerArray, we create a few markers
                visualization_msgs::MarkerArray marker_array;
                marker_array.markers.push_back(produceMarker(temparray.poses[0],color,marker_counter));
                viz_pub_.publish(marker_array);
                publishPose(temparray);

                temparray.poses[0] = array.poses[1];
                color.a=1.0;
                color.r=0;
                color.g=1.0; 
                color.b=0.0;    

                marker_counter=0;
                //Let's also publish the markerArray, we create a few markers
                marker_array.markers.clear();
                marker_array.markers.push_back(produceMarker(temparray.poses[0],color,marker_counter));
                viz_pub_.publish(marker_array);
                publishPose(temparray);

            }
        }
        if(temp != pose)
        {
            count = 0;
            following_ = true;
            beside_ = false;
            interrupt_ = false;
        }
        temp = pose;
        limiter.sleep();
    }

}

geometry_msgs::PoseArray FnFadv::standbeside()
{
    beside_ = true;
    //interrupt_ = true;
    
    geometry_msgs::PoseArray array;
    geometry_msgs::PoseArray poses;
    geometry_msgs::Pose point_beside_l; //goal pose on left of r1
    geometry_msgs::Pose point_beside_r; //goal pose on right of r1
    geometry_msgs::Pose interim_l;     //respective interim points to avoid colliding
    geometry_msgs::Pose interim_r;
    geometry_msgs::Point pb_local;
    nav_msgs::OccupancyGrid grid;
    geometry_msgs::Pose r1;
    geometry_msgs::Pose r0;
    int side = 0;
    
    mtx_.lock();
    r0 = data_.pose0;
    r1 = data_.pose1;
    grid = Grid_;
    mtx_.unlock();
    GridProcessing gridprocc(grid);

    pb_local.x = 0;
    pb_local.y = -1.0;
    point_beside_r.position = localtoglobal(r1,pb_local);  //get point to right of r1
    point_beside_r.orientation = r1.orientation;           //copy r1's orientation
    pb_local.y = 1.0;
    point_beside_l.position = localtoglobal(r1,pb_local);   //get point to left of r1
    point_beside_l.orientation = r1.orientation;

    pb_local.x = -1.0;
    pb_local.y = -1.0;
    interim_r.position = localtoglobal(r1,pb_local);        //get a point to diagonally to the right and behind of r1 for the case that r0 going directly to r1s side causes a collision
    double yaw = atan2((point_beside_r.position.y-interim_r.position.y),(point_beside_r.position.x-interim_r.position.x));
    if(yaw<0){yaw = 2*M_PI + yaw;}
    //if(yaw==0 && current.position.x>inter_point.position.x){yaw = M_PI;}
    YawtoQuat(yaw,interim_r);
    
    
    pb_local.y = 1.0;
    interim_l.position = localtoglobal(r1,pb_local);    //point diagonally to left and behind of r1
    yaw = atan2((point_beside_l.position.y-interim_l.position.y),(point_beside_l.position.x-interim_l.position.x));
    if(yaw<0){yaw = 2*M_PI + yaw;}
    //if(yaw==0 && current.position.x>inter_point.position.x){yaw = M_PI;}
    YawtoQuat(yaw,interim_l);

    geometry_msgs::Point corner1;
    geometry_msgs::Point corner2;

    corner1.x = diameter/2;
    corner1.y = diameter/2;
    corner2.x = diameter/2;
    corner2.y = -1*diameter/2;

    bool cornercheck1;
    bool cornercheck2;
    
    checkCorners(r0,point_beside_r,corner1,corner2,gridprocc,cornercheck1,cornercheck2); //check if r0 can go to right of r1 without colliding
    if(cornercheck1 && cornercheck2)    //if yes go there
    {
        //array.poses.push_back(point_beside_r);
        poses.poses.push_back(point_beside_r);
        // std_msgs::ColorRGBA color;
        // color.a=1.0;//a is alpha - transparency 0.5 is 50%;
        // color.r=0;
        // color.g=1.0; 
        // color.b=0;   

        // int marker_counter=0;
        // //Let's also publish the markerArray, we create a few markers
        // visualization_msgs::MarkerArray marker_array;
        // marker_array.markers.push_back(produceMarker(array.poses[0],color,marker_counter));
        // viz_pub_.publish(marker_array);
        // publishPose(array);
        side = 23;      //random number to avoid future if conditions
        //array.poses.clear();
    }
    if(side != 23)
    {
        checkCorners(r0,point_beside_l,corner1,corner2,gridprocc,cornercheck1,cornercheck2); //if right failed check left
        if(cornercheck1 && cornercheck2)
        {
            //array.poses.push_back(point_beside_l);
            poses.poses.push_back(point_beside_l);
            // std_msgs::ColorRGBA color;
            // color.a=1.0;//a is alpha - transparency 0.5 is 50%;
            // color.r=0;
            // color.g=1.0; 
            // color.b=0;   

            // int marker_counter=0;
            // //Let's also publish the markerArray, we create a few markers
            // visualization_msgs::MarkerArray marker_array;
            // marker_array.markers.push_back(produceMarker(array.poses[0],color,marker_counter));
            // viz_pub_.publish(marker_array);
            // publishPose(array);
            side = 23;      //random number to avoid future if conditions
            //array.poses.clear();
        }
    }
    if(side != 23) //if neither left or right could be directly reached check interim points
    {
        checkCorners(r0,interim_r,corner1,corner2,gridprocc,cornercheck1,cornercheck2);
        if(cornercheck1 && cornercheck2)
        {
            geometry_msgs::Pose ext_temp;   //extrapolated pose to if r0 is at the interim point coz we now want to check if the path between the interim point and the point beside r1 is free
            ext_temp.position = interim_r.position;
            checkCorners(ext_temp,point_beside_r,corner1,corner2,gridprocc,cornercheck1,cornercheck2);
            if(cornercheck1 && cornercheck2)
            {
                side = 1;
            }
        }
    }
    if(side == 0)
    {
        checkCorners(r0,interim_l,corner1,corner2,gridprocc,cornercheck1,cornercheck2);
        if(cornercheck1 && cornercheck2)
        {
            geometry_msgs::Pose ext_temp;   //extrapolated pose to if r0 is at the interim point coz we now want to check if the path between the interim point and the point beside r1 is free
            ext_temp.position = interim_r.position;
            checkCorners(ext_temp,point_beside_l,corner1,corner2,gridprocc,cornercheck1,cornercheck2);
            if(cornercheck1 && cornercheck2)
            {
                side = 2;
            }
        }
    }
    if(side == 0)
    {
        ROS_WARN_STREAM("Cannot reach either side. Staying put.");
    }
    
    if(side == 1)
    {
        //array.poses.push_back(interim_r);
        poses.poses.push_back(interim_r);
        // std_msgs::ColorRGBA color;
        // color.a=1.0;//a is alpha - transparency 0.5 is 50%;
        // color.r=0;
        // color.g=0; 
        // color.b=1.0;    //it will be blue - for interim pose

        // int marker_counter=0;
        // //Let's also publish the markerArray, we create a few markers
        // visualization_msgs::MarkerArray marker_array;
        // marker_array.markers.push_back(produceMarker(array.poses[0],color,marker_counter));
        // viz_pub_.publish(marker_array);
        // publishPose(array);

        //array.poses[0] = point_beside_r;
        poses.poses.push_back(point_beside_r);
        // color.a=1.0;
        // color.r=0;
        // color.g=1.0; 
        // color.b=0.0;    

        // marker_counter=0;
        // //Let's also publish the markerArray, we create a few markers
        // marker_array.markers.clear();
        // marker_array.markers.push_back(produceMarker(array.poses[0],color,marker_counter));
        // viz_pub_.publish(marker_array);
        // publishPose(array);
    }
    if(side == 2)
    {
        //array.poses.push_back(interim_l);
        poses.poses.push_back(interim_l);
        // std_msgs::ColorRGBA color;
        // color.a=1.0;//a is alpha - transparency 0.5 is 50%;
        // color.r=0;
        // color.g=0; 
        // color.b=1.0;    //it will be blue - for interim pose

        // int marker_counter=0;
        // //Let's also publish the markerArray, we create a few markers
        // visualization_msgs::MarkerArray marker_array;
        // marker_array.markers.push_back(produceMarker(array.poses[0],color,marker_counter));
        // viz_pub_.publish(marker_array);
        // publishPose(array);

        //array.poses[0] = point_beside_l;
        poses.poses.push_back(point_beside_l);
        // color.a=1.0;
        // color.r=0;
        // color.g=1.0; 
        // color.b=0.0;    

        // marker_counter=0;
        // //Let's also publish the markerArray, we create a few markers
        // marker_array.markers.clear();
        // marker_array.markers.push_back(produceMarker(array.poses[0],color,marker_counter));
        // viz_pub_.publish(marker_array);
        // publishPose(array);
    }
    return poses;
}

void FnFadv::distancecheck()
{
    ros::Duration(2.0).sleep();
    FnFData data;
    geometry_msgs::Pose current;
    geometry_msgs::Pose current1;
    geometry_msgs::PoseArray array;
    std::unique_lock<std::mutex> lck(Followmtx_);
    Followcv_.wait(lck,[&](){return following_ == true;});
    lck.unlock();
    while(ros::ok())
    {
        mtx_.lock();
        data = data_;
        mtx_.unlock();
        current = data.pose0;
        current1 = data.pose1;

        double dist_t_r1 = pow(pow((current1.position.y-current.position.y),2) + pow((current1.position.x - current.position.x),2),0.5);
        //ROS_INFO_STREAM("Dist: "<<dist_t_r1);
        if(dist_t_r1<0.6 && !beside_)                     //if too close dont do anything
        {
            array.poses.clear();
            ROS_WARN_STREAM("Too close! Abandoning current goal");
            publishPose(array);
        }    
    }
}