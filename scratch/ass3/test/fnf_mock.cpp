#include "fnf_mock.h"

FnF_mock::FnF_mock()
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

void FnF_mock::setData(FnFData data, nav_msgs::OccupancyGrid grid)
{
    std::unique_lock<std::mutex> lck(mtx_);
    data_ = data;
    lck.unlock();
    Grid_ = grid;
    datagot_ = true;
    cv_.notify_all();

}
bool FnF_mock::IsinLoS()
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


geometry_msgs::PoseArray FnF_mock::getpath(Pathpacket pack)
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
        return path;
        
    }
    if(!reachable1 || !reachable2) //if one corner cannot reach goal, need new goal
    {
        //ROS_WARN_STREAM("Cannot reach D:! Rerouting!");
        path = goaround(pack,corner1,corner2);
        return path;
    }   
    
}

geometry_msgs::PoseArray FnF_mock::goaround(Pathpacket pack,geometry_msgs::Point corner1,geometry_msgs::Point corner2)
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
    return array;
}


geometry_msgs::PoseArray FnF_mock::standbeside()
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
        side = 23;      //random number to avoid future if conditions
    }
    if(side != 23)
    {
        checkCorners(r0,point_beside_l,corner1,corner2,gridprocc,cornercheck1,cornercheck2); //if right failed check left
        if(cornercheck1 && cornercheck2)
        {
            poses.poses.push_back(point_beside_l);
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
        //ROS_WARN_STREAM("Cannot reach either side. Staying put.");
    }
    
    if(side == 1)
    {
        poses.poses.push_back(interim_r);
        poses.poses.push_back(point_beside_r);
    }
    if(side == 2)
    {
        poses.poses.push_back(interim_l);
        poses.poses.push_back(point_beside_l);
    }
    return poses;

}