#include "findnfollow_int.h"

void FnFint::setData(FnFData data, nav_msgs::OccupancyGrid grid)
{
    std::unique_lock<std::mutex> lck(mtx_);
    data_ = data;
    lck.unlock();
    Grid_ = grid;
    datagot_ = true;
    cv_.notify_all();
}

void FnFint::setHandle(ros::NodeHandle nh)
{
    nh_ = nh;
    pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("robot_0/path",10,false);
    //
    viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("robot_0/following",3,false);
}


void FnFint::search()
{
    //while(ros::ok())
    //{
        geometry_msgs::Pose pose;
        geometry_msgs::PoseArray poses;
        double yaw = 0;
        int count = 0;
        ros::Rate rate_limiter(1.0/3.5);
        std::unique_lock<std::mutex> lck(mtx_);
        cv_.wait(lck,[&](){return datagot_ == true && searching_ == true;});
        geometry_msgs::Pose p = data_.pose0;
        lck.unlock();
        yaw = tf::getYaw(p.orientation);
        IsinLoS();
        while(!insight_)
        {
          yaw = yaw + M_PI;
          YawtoQuat(yaw,pose);
          pose.position = p.position;
          poses.poses = {pose};
          pose_pub_.publish(poses); //rotates r0 on the spot to search for r1
          moving_ = true;
          moving_ = false;
          rate_limiter.sleep();
          IsinLoS();
        }
        Followcv_.notify_all();
    //}

}


bool FnFint::IsinLoS() 
{
    if(!moving_)
{
    mtx_.lock();
    FnFData data = data_;
    mtx_.unlock();
    geometry_msgs::Point xy1;
    geometry_msgs::Point xy0;
    geometry_msgs::Point r1_local;

    double orient0;
    double angle;
    xy1.x = data.pose1.position.x;
    xy1.y = data.pose1.position.y;
    orient0 = tf::getYaw(data.pose0.orientation);
    if(orient0<0){orient0 += 2*M_PI;}
    xy0.x = data.pose0.position.x;
    xy0.y = data.pose0.position.y;

//get position of r1 in r0s local frame of reference
    r1_local.x = (xy1.x - xy0.x)*cos(orient0) + (xy1.y - xy0.y)*sin(orient0); 
    r1_local.y = -(xy1.x - xy0.x)*sin(orient0) + (xy1.y - xy0.y)*cos(orient0);

//calculate the distance between r0 and r1 along with the bearing to r1 from r0
    double dist = pow((pow((r1_local.x),2) + pow((r1_local.y),2)),0.5);
    double bearing = atan2(r1_local.y,r1_local.x);
    if(r1_local.x<0)    //if r1's position is behind r0 then it cant be in line of sight
    {
        insight_ = false;
        searching_ = true;
        following_ = false;
        R1mtx_.lock();
        R1poses_.poses.clear();
        R1mtx_.unlock();
        return false;
    }
    else
    {
        if(bearing<0) //determine the angle to r1 in r0s frame of reference in terms of the laser readings, i.e. in between -90 and +90 degrees from the +x axis
        {
            angle = M_PI/2 - (bearing*-1);
        }
        if(bearing>0)
        {
            angle = M_PI/2 + bearing;
        }
        int rng_num = round(angle/data.laserscan.angle_increment); //determine the index within the laser scan based on the angle calculated above
        int i = 0;
        int j = 0;
//Along with the above point, get a few more points around that point to cover a wider range of readings to avoid false negatives
        if(rng_num>5)
        {
            i = rng_num - 5;
            j = rng_num + 5;
        }
        if(rng_num<5)
        {
            i = rng_num - 1;
            j = rng_num + 5;
        }
        if(rng_num>data.laserscan.ranges.size()-5)
        {
            j = rng_num;
            i = rng_num - 5;
        }
        if(rng_num<data.laserscan.ranges.size()-5)
        {
            j = rng_num + 5;
            i = rng_num - 5;
        }
//////////////////////////////////////////////
        bool check;
        for(i;i<j;i++)
        {
            if(abs(data.laserscan.ranges[i]-dist)<0.2) //if the distance calculated and the laser reading at the index are similar within a range of 0.2
            {
                if(data.laserscan.intensities[i]==1)    //if the reading's intensity is 1
                {
                    check = true;           //then the laser reading is landing on r1 and this r1 is in line of sight
                    break;
                }
                else
                {
                    ROS_INFO_STREAM("SEARCHING!");
                    check = false;
                }
            }
            else
            {
                check = false;
            }
        }
        if(check)
        {
            insight_ = true;
            searching_ = false;
            following_ = true;
            return true;
        }
        else
        {
            insight_ = false;
            searching_ = true;
            following_ = false;
            R1mtx_.lock();
            R1poses_.poses.clear();
            R1mtx_.unlock();
            return false;
        }
    }
}
}

void FnFint::publishPose(geometry_msgs::PoseArray pose)
{
    pose_pub_.publish(pose);
    
    mtx_.lock();
    geometry_msgs::Pose current = data_.pose0;
    mtx_.unlock();
    //moving_ = true;
    if(!pose.poses.empty())
    {
        geometry_msgs::Point temp = pose.poses.back().position;
        geometry_msgs::Quaternion tempq = pose.poses.back().orientation;
        //wait for r0 to reach goal pose before being able to publish another one
        while(abs(current.position.x-temp.x)>0.1 || abs(current.position.y-temp.y)>0.1
            || (abs(current.orientation.w)-abs(tempq.w))>0.1 || (abs(current.orientation.z)-abs(tempq.z))>0.1)
        {
            if(interrupt_)
            {break;}
        
            mtx_.lock();
            current = data_.pose0;
            mtx_.unlock();
        }
    }
    //moving_ = false;
    //lck.unlock();
    
}




geometry_msgs::PoseArray FnFint::getpath(Pathpacket pack)
{
    geometry_msgs::PoseArray path;
    // geometry_msgs::Pose current = pack.current0;
    geometry_msgs::Pose goal = pack.point_around1;

    bool reachable1;
    bool reachable2;

    geometry_msgs::Point corner1;
    geometry_msgs::Point corner2;

    corner1.x = diameter/2;
    corner1.y = diameter/2;
    corner2.x = diameter/2;
    corner2.y = -1*diameter/2;

    mtx_.lock();
    nav_msgs::OccupancyGrid grid = Grid_;
    GridProcessing gridProcessing(grid);
    mtx_.unlock();

    checkCorners(pack.current0,goal,corner1,corner2,gridProcessing,reachable1,reachable2); //check if r0s corners can reach the goal unhindered

    if(reachable1 && reachable2) //if both corners can reach goal, all good to go
    {
        path.poses = {goal};
        
        std_msgs::ColorRGBA color;
        color.a=1.0;
        color.r=0;
        color.g=1.0; // it will be green - final pose before facing r1
        color.b=0;

        int marker_counter=0;
        
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.push_back(produceMarker(goal,color,marker_counter));
        viz_pub_.publish(marker_array);
        return path;
        
    }
    if(!reachable1 || !reachable2) //if one corner cannot reach goal, need new goal
    {
        ROS_WARN_STREAM("Cannot reach D:! Rerouting!");
        path = goaround(pack,corner1,corner2);
        return path;
    }   
    
}

geometry_msgs::PoseArray FnFint::goaround(Pathpacket pack,geometry_msgs::Point corner1,geometry_msgs::Point corner2)
{
    geometry_msgs::PoseArray array;
    bool reachable1 = false;
    bool reachable2 = false;
    bool horizontal;    //bools to tell if r1 is in a horizontal/vertical/angled corridor
    bool vertical;
    bool angled;        //
    int corridor = 0;
    geometry_msgs::Pose current = pack.current0;
    geometry_msgs::Pose r1pose = pack.current1;
    geometry_msgs::Point zero;
    geometry_msgs::Pose inter_point; //since goal isnt directly reachable, we look for an intermediary point

    geometry_msgs::Point temp;
    geometry_msgs::Point temp_goal;
    temp.x = current.position.x;
    temp.y = r1pose.position.y;

    //convert points for checking in Ogmap
    zero.x = temp.x - current.position.x;
    zero.y = temp.y - current.position.y;

 
    temp_goal.x = r1pose.position.x - current.position.x;   
    temp_goal.y = r1pose.position.y - current.position.y;

    mtx_.lock();
    nav_msgs::OccupancyGrid grid = Grid_;
    GridProcessing gridProcessing(grid);
    mtx_.unlock();
    

    bool type = Corridortype(zero,temp_goal,1,gridProcessing); //check if robot1 is in a horizontal corridor

    if(type) 
    {
        corridor = 1;                           //corridor = 1 means horizontal for switch case
    }
    else                                        //now check if in vertical corridor
    {
        temp.x = r1pose.position.x;
        temp.y = current.position.y;
        zero.x = temp.x - current.position.x;
        zero.y = temp.y - current.position.y;
        type = Corridortype(zero,temp_goal,2,gridProcessing); //now check if in vertical corridor
        if(type)
        {
            corridor = 2;                           //corridor = 2 means vertical for switch case
        }
        else                                        //if neither is true then must be angled
        {
            corridor = 3;
        }
    }
    geometry_msgs::Point point_check;
    point_check.x = 0;  //start at x=0, 0 because robot is always at origin of ogmap
    point_check.y = 0;  //start at x=0, 0 because robot is always at origin of ogmap
    int loop_num = 0;
    while(!reachable1 || !reachable2) 
    {
        mtx_.lock();
        nav_msgs::OccupancyGrid grid = Grid_;
        GridProcessing gridProcessing(grid);
        mtx_.unlock();
        gridProcessing.setGrid(grid);
        switch (corridor)       //corridor type
        {
        case 1:                 //horizontal - keep y same, change x
            inter_point.position.y = r1pose.position.y;// - current.position.y;
            inter_point.position.x = point_check.x + current.position.x;       
            point_check.y = current.position.y;
            checkCorners(current,inter_point,corner1,corner2,gridProcessing,reachable1,reachable2);
            break;
        case 2:                 //vertical - keep x same, change y
            inter_point.position.x = r1pose.position.x;// - current.position.x;
            inter_point.position.y = point_check.y + current.position.y;       
            point_check.x = current.position.x;
            checkCorners(current,inter_point,corner1,corner2,gridProcessing,reachable1,reachable2);
            break;
        case 3:                //change both x and y
            inter_point.position.y = point_check.y + current.position.y;
            inter_point.position.x = point_check.x + current.position.x; 
            checkCorners(current,inter_point,corner1,corner2,gridProcessing,reachable1,reachable2);
            break;

        default:
            break;
        }
        
        zero.x = 0;
        zero.y = 0;
        if(reachable1 && reachable2)
        {
            double yaw = atan2((inter_point.position.y-current.position.y),(inter_point.position.x-current.position.x));
            if(yaw<0){yaw = 2*M_PI + yaw;}
            if(yaw==0 && current.position.x>inter_point.position.x){yaw = M_PI;} //if goal is on the left, we want orient of 180 not 0, atan2 does not differientiate here so we must include this if
            
            YawtoQuat(yaw,inter_point);
            array.poses.push_back(inter_point);
        }
        else if(!reachable1 || !reachable2) 
        {
            switch (corridor)
            {
            case 1:             //change x point
                if(current.position.x<r1pose.position.x)    //if r1 is to right, move x left
                {
                    point_check.x = point_check.x - 0.2;   
                }
                else                                        //if r1 is to left, move x right
                {
                    point_check.x = point_check.x + 0.2;
                }
                break;
            case 2:             //change y point
                if(current.position.y<r1pose.position.y)
                {
                    point_check.y = point_check.y - 0.2;    //if r1 is above, move y down
                }    
                else
                {
                    point_check.y = point_check.y + 0.2;    //if r1 is below, move y up
                }
                break;
            case 3:             //change x and y point
                if(current.position.x<r1pose.position.x)    //if r1 is to right, move x left
                {
                    if(current.position.y<r1pose.position.y)    //if r1 is above, & to right, move y up
                    {
                        point_check.x = point_check.x - 0.2; 
                        point_check.y = point_check.y + 0.2;
                    }
                    else
                    {
                        point_check.x = point_check.x - 0.2;    //else move y down
                        point_check.y = point_check.y - 0.2;  
                    }               
                }
                else                                            //if r1 is to left, move x right
                {
                    if(current.position.y<r1pose.position.y)    //if r1 is above, move y up   
                    {
                        point_check.x = point_check.x + 0.2;
                        point_check.y = point_check.y + 0.2;
                    }
                    else                                        //if r1 is below, move y down
                    {
                        point_check.x = point_check.x + 0.2; 
                        point_check.y = point_check.y - 0.2;
                    }
                }
                break;
            
            default:
                break;
            }
        }
        loop_num = loop_num + 1;
    }
    
     //We publish the point to screen using a function
    std_msgs::ColorRGBA color;
    color.a=1.0;//a is alpha - transparency 0.5 is 50%;
    color.r=0;
    color.g=0; 
    color.b=1.0;    //it will be blue - for interim pose

    int marker_counter=0;
    //Let's also publish the markerArray, we create a few markers
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(produceMarker(array.poses[0],color,marker_counter));
    viz_pub_.publish(marker_array);
    return array;
}

visualization_msgs::Marker FnFint::produceMarker(geometry_msgs::Pose point, std_msgs::ColorRGBA color,int& id)
{
    visualization_msgs::Marker marker;

    //We need to set the frame
    // Set the frame ID and time stamp.
    marker.header.frame_id = "world";
    //single_marker_person.header.stamp = ros::Time();
    marker.header.stamp = ros::Time::now();


    //We set lifetime (it will dissapear in this many seconds)
    marker.lifetime = ros::Duration(4.0);
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "test";
    marker.id = id++;

    // The marker type
    marker.type = visualization_msgs::Marker::ARROW;

    // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position = point.position;

    //Orientation, we are going to leave it as 0,0,0,1 for quaternion,
    //For an arrow you copudl consider passing yaw or quaternion for it
    marker.pose.orientation = point.orientation;

    // Set the scale of the marker -- 0.5x0.5x0.5 here means 0.5m side
    marker.scale.x = 1.0;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color = color;

    return marker;
}


void FnFint::checkLoS()
{
    ros::Duration(2.0).sleep();
    ros::Rate limiter(1/1.0); //1Hz
    geometry_msgs::Pose temp;
    geometry_msgs::PoseArray array;
    double count = 0;
    bool check;
    std::unique_lock<std::mutex> lck(Followmtx_);
    Followcv_.wait(lck,[&](){return following_ == true;});
    lck.unlock();
    while(ros::ok())
    {
       
            moving_ = false;
            check = IsinLoS();       
            if(check)
            {
                count = 0;
                following_ = true;
                lost_ = false;
            }
            if(!check)
            {
                count = count + 1;
            }
            if(count>10)
            {
                following_ = false;
                interrupt_ = true;
                array.poses.clear();
                pose_pub_.publish(array);
                interrupt_ = false;
                lost_ = true;
                if(count == 11)
                {
                    ROS_WARN_STREAM("Where did you go? I'll wait here.");
                }
            }
        
        limiter.sleep();
    }

}