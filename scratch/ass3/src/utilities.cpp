#include "utilities.h"

void YawtoQuat(double yaw, geometry_msgs::Pose& pose)
{
    tf::Quaternion quat = tf::createQuaternionFromYaw(yaw);
    pose.orientation.w = quat.getW();
    pose.orientation.x = quat.getX();
    pose.orientation.y = quat.getY();
    pose.orientation.z = quat.getZ();
}

geometry_msgs::Point getpointaroundR1(geometry_msgs::Point r0, geometry_msgs::Point r1)
{
    
    std::vector<geometry_msgs::Point> radial;
    geometry_msgs::Point p;
    for(int i=0;i<16;i++)       //get 16 equally spaced points along the circle around r1
    {
        p.x = r1.x + 1*cos(2*M_PI-(M_PI/180)*(i*22.5));
        p.y = r1.y + 1*sin(M_PI-(M_PI/180)*(i*22.5));
        radial.push_back(p);
    }
    double temp = 1000;
    double dist = 0;
    int id = 0;
    for(int i=0;i<radial.size();i++)    //find the closest point to r0 from those 16 points
    {
        double x = r0.x-radial[i].x; 
        double y = r0.y-radial[i].y; 
        dist = pow((pow((x),2) + pow((y),2)),0.5);
        if(dist<temp)
        {
            id = i;
            temp = dist;
        }   
    }

    return radial[id];      //return the closest one
}

geometry_msgs::Point getpointbehindR1(geometry_msgs::Pose r1)
{
    
    double yaw = 0;
    geometry_msgs::Point point_behind;
    geometry_msgs::Pose pose_behind;
    geometry_msgs::Quaternion q = r1.orientation;

    yaw = atan2(2.0*(q.w*q.z + q.y*q.x), q.w*q.w - q.x*q.x - q.y*q.y - q.z*q.z);
    if(yaw<0){yaw += 2*M_PI;}
    //we use 0.7 to account for the size of the robots
    point_behind.x = (-0.7)*cos(yaw) - (0)*sin(yaw) + r1.position.x;
    point_behind.y = (-0.7)*sin(yaw) + (0)*cos(yaw) + r1.position.y;
    
   
    //pose_behind.orientation = data.pose1.orientation;
    return point_behind;  

}

geometry_msgs::Point localtoglobal(geometry_msgs::Pose current, geometry_msgs::Point point)
{
    geometry_msgs::Point cvtpt;
    double x0 = current.position.x;
    double y0 = current.position.y;
    double yaw = tf::getYaw(current.orientation);
    if(yaw<0){yaw = yaw + 2*M_PI;}
    cvtpt.x = point.x*cos(yaw) - point.y*sin(yaw) + x0;
    cvtpt.y = point.x*sin(yaw) + point.y*cos(yaw) + y0;
    return cvtpt;
}


void checkCorners(geometry_msgs::Pose current, geometry_msgs::Pose goal, geometry_msgs::Point corner1, geometry_msgs::Point corner2, GridProcessing& grid,bool& reachable1,bool& reachable2)
{
    geometry_msgs::Point corner1c;
    geometry_msgs::Point corner2c;
    geometry_msgs::Point corner1g;
    geometry_msgs::Point corner2g;
    double orient = atan2((goal.position.y-current.position.y),(goal.position.x-current.position.x)); //orientation if r0 is facing goal from current pos
    if(orient<0){orient = orient + 2*M_PI;}
    if(orient==0 && current.position.x>goal.position.x){orient = M_PI;} //if goal is on the left, we want orient of 180 not 0, atan2 does not differientiate here so we must include this if
    geometry_msgs::Pose temp = current;
    YawtoQuat(orient,temp);                 //rotate the orientation so that r0 is theoretically facing its goal
    corner1c = localtoglobal(temp,corner1); //global loc of corner at current pos facing goal
    corner2c = localtoglobal(temp,corner2);
    temp.position = goal.position;
    corner1g = localtoglobal(temp,corner1); //global loc of corners at goal pos
    corner2g = localtoglobal(temp,corner2);

    geometry_msgs::Point goal_local;
    geometry_msgs::Point zero;
    zero.x = 0; //centre of robot
    zero.y = 0;   
    //bool reachable = gridProcessing.checkConnectivity(zero,goal_local);

    zero.x = corner1c.x - current.position.x; //corner1 of robot at current pos in ogmap
    zero.y = corner1c.y - current.position.y; 
    goal_local.x = corner1g.x - current.position.x;   //corner1 of robot at goal pos in ogmap
    goal_local.y = corner1g.y - current.position.y;
    reachable1 = grid.checkConnectivity(zero,goal_local); //check if corner1 can go from current pos to goal pos without passing through occupied cells

    zero.x = corner2c.x - current.position.x; //corner2 of robot
    zero.y = corner2c.y - current.position.y; 
    goal_local.x = corner2g.x - current.position.x;   //corner2 of robot at goal pos in ogmap
    goal_local.y = corner2g.y - current.position.y;
    reachable2 = grid.checkConnectivity(zero,goal_local);   //check if corner2 can go from current pos to goal pos without passing through occupied cells
}


bool Corridortype(geometry_msgs::Point zero, geometry_msgs::Point temp_goal,int check_type,GridProcessing& grid)
{
    //zero //interm_points
    //temp_goal// r1pose - doesnt change

    double count = 0;
    double t = 0;

    switch (check_type)
    {
    case 1:                         //check for horizontal corridor
        zero.y = zero.y - 0.5;      //get 20 points along axis with centre point being of r1 to avoid gaps in ogmap
        for(int i=0;i<20;i++)
        {
            zero.y = zero.y + 0.05;
            if(!grid.checkConnectivity(zero,temp_goal)) //check if that point can get to r1
            {
                count = count + 1;
                t = t + 1;
                //ROS_INFO_STREAM("Trues: "<<t);
            }
            else
            {
                count = count + 1;
            }
        }           
        break;
    case 2:
        zero.x = zero.x - 0.5;
            for(int i=0;i<10;i++)
            {
                zero.x = zero.x + 0.1;
                if(!grid.checkConnectivity(zero,temp_goal))
                {
                    count = count + 1;
                    t = t + 1;
                }
                else
                {
                    count = count + 1;
                }
            } 
        break;
    default:
        break;
    }
    double per;
    per = t/count;
    //ROS_INFO_STREAM("Per: "<<per<<" Count: "<<count);
    if(per>0.55)        //if more than 55% of the points cannot reach r1 then it must not be the type we were checking for
    {
        return false;
    }
    else 
    {
        return true;
    }
}