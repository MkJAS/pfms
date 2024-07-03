#include "laserprocessing.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include "tf/transform_datatypes.h"


LaserProcessing::LaserProcessing(sensor_msgs::LaserScan laserScan):
    laserScan_(laserScan)
{

}



//! @todo
//! TASK 1 - Refer to README.md and the Header file for full description
unsigned int LaserProcessing::countHighIntensity()
{
  unsigned int count=0;
  std::vector<float> temp = laserScan_.intensities;
 
  for(auto it:temp)
  {
      if(it>0.5)
      {count++;}
  }
  return count;
}

//! @todo
//! TASK 2 - Refer to README.md and the Header file for full description
unsigned int LaserProcessing::countSegments()
{
  unsigned int count=0;
  std::vector<float> ints = laserScan_.intensities;
  std::vector<float> rngs = laserScan_.ranges;
  double angle = 0;
  geometry_msgs::Point point1;
  geometry_msgs::Point point2;
  double dist;
  bool check = true;
  int con = 0;
  for(int i=0;i<ints.size()-1;i++)
  {
      if(ints[i]>0.5)
      {
          point1 = polarToCart(i);
          for(int j=i;j<ints.size();j++)
          {
            if(ints[j+1]<0.5)
            {
                break;
            }  
            if(ints[j+1]>0.5)
            {
                point2 = polarToCart(j+1);
                dist = pow((pow((point2.x-point1.x),2) + pow((point2.y-point1.y),2)),0.5);
                if(dist<0.3)
                {
                    con++;
                }
                else {break;}
            }
          }
      }
      if(con>0)
      {
          count++;
          con = 0;
      }
  }
  return count;
}



//! @todo
//! TASK 3 - Refer to README.md and the Header file for full description
geometry_msgs::Point LaserProcessing::detectPositionHighIntensity(){

    geometry_msgs::Point point;
    //ROS_INFO_STREAM("AAAAAAAAAAAAA");
    std::vector<float> ints = laserScan_.intensities;
    std::vector<float> rngs = laserScan_.ranges;
    double x,y;

    for(int i=0;i<ints.size();i++)
    {
        if(ints[i]>0.5)
        {
            point = polarToCart(i);
            break;
        }
    }
    // double angle = id* (laserScan_.angle_increment);
    // double x = min*cos(2*M_PI-(angle-M_PI/2+yaw_));
    // double y = min*sin(M_PI-(angle-M_PI/2+yaw_));

    return point;
}


//! @todo
//! TASK 4 - Refer to README.md and the Header file for full description
geometry_msgs::Pose LaserProcessing::detectPoseHighIntensity()
{
    geometry_msgs::Pose pose;
    std::vector<float> ints = laserScan_.intensities;
    //std::vector<float> rngs = laserScan_.ranges;
    std::vector<geometry_msgs::Point> points;
    geometry_msgs::Point point2;
    geometry_msgs::Point point1;
    double dist = 0;
    while(points.size()!=4)
    {
    for(int i=0;i<ints.size()-1;i++)
    {
      if(ints[i]>0.5)
      {
          point1 = polarToCart(i);
          points.push_back(point1);
          for(int j=i;j<ints.size();j++)
          {
            if(ints[j+1]<0.5)
            {
                break;
            }  
            if(ints[j+1]>0.5)
            {
                point2 = polarToCart(j+1);
                dist = pow((pow((point2.x-point1.x),2) + pow((point2.y-point1.y),2)),0.5);
                if(dist<0.3)
                {
                    points.push_back(point2);
                }
                else {break;}
            }
          }
      }
    }
    }
    double angle = angleConnectingPoints(points[0],points[3]);
    if(angle<0){angle+2*M_PI;}
    tf::Quaternion quat = tf::createQuaternionFromYaw(angle);
    pose.orientation.w = quat.getW();
    pose.orientation.x = quat.getX();
    pose.orientation.y = quat.getY();
    pose.orientation.z = quat.getZ();
    pose.orientation = tf::createQuaternionMsgFromYaw(angle);


    return pose;
}

void LaserProcessing::newScan(sensor_msgs::LaserScan laserScan){
    laserScan_=laserScan;
}


geometry_msgs::Point LaserProcessing::polarToCart(unsigned int index)
{
    float angle = laserScan_.angle_min + laserScan_.angle_increment*index;// + angle_range/2;
    float range = laserScan_.ranges.at(index);
    geometry_msgs::Point cart;
    cart.x = static_cast<double>(range*cos(angle));
    cart.y = static_cast<double>(range*sin(angle));
    return cart;
}

double LaserProcessing::angleConnectingPoints(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    return atan2(p2.y - p1.y, p2.x - p1.x);
}
