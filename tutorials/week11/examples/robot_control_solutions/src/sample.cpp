
#include "sample.h"

/**
 * This sample code is provided to illustrate
 * - Subscribing to standard topics (Odometry and Laser)
 * - Publishing velocity commands
 */

RobotControl::RobotControl(ros::NodeHandle nh)
    : nh_(nh), tooClose(false),stationary(false)
{
  sub1_ = nh_.subscribe("/robot_1/base_scan", 10, &RobotControl::laserCallback, this);
  sub2_ = nh_.subscribe("/robot_1/cmd_vel", 10, &RobotControl::cmdVelCallback, this);

  //Publish a velocity ... to control the robot
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel", 1);
}

RobotControl::~RobotControl()
{
}

void RobotControl::laserCallback(const sensor_msgs::LaserScanConstPtr &msg)
{

  //!
  //! @todo : 1 - Detect when the robot is closer than 0.15m to an obstacle
  //!
  tooClose = false;
  for (auto i = 0; i < msg->ranges.size(); i++)
  {
    if (msg->ranges[i] < 0.25)
    {
      // this will execute when im closer then 0.15m but it has not been speciifed what to do when that happens
      ROS_INFO_STREAM("TO CLOSE BACK UP!");
      tooClose=true;
      break;
    }
  }
}

void RobotControl::cmdVelCallback(const geometry_msgs::TwistConstPtr &msg)
{
  static double lastangle;
  static std::pair<double, double> lastpos;
  //!
  //! @todo : 2 - Detect when the robot is stationary
  //!
  //! - Expect zero for linear x and angular z
  //!
  //!
  if ( (msg->angular.z<0.01)  && (msg->angular.z>-0.01) && (msg->linear.x<0.01) && (msg->linear.x>-0.01))
  {
    // this will be true when the robot is not moving or turning
    ROS_INFO_STREAM("Robot stopped");
    stationary=true;
  }
  else {
    stationary=false;
  }
}

void RobotControl::seperateThread()
{
  /**
    * The below loop runs until ros is shutdown
    */
  //We need two timers
  ros::Time beginDetection;
  ros::Time beginReversing;

  bool startedDetectionTimer = false;
  bool startReversing = false;

  ros::Rate rate_limiter(10);
  while (ros::ok())
  {

    //! @todo : 3 - Check that the object is within 0.15m and the robot has been statioray for more than 3 seconds
    //!
    //! To do the check, we need to see the robot is stationary from the first time he has stopped (velocity zero)
    //! REFER : http://wiki.ros.org/roscpp/Overview/Time

    if(stationary && tooClose){
      if(startedDetectionTimer){
        //Let's check elapsed time
        ros::Duration d = ros::Time::now() - beginDetection;
        if(d.toSec()>3&&(!startReversing)){
          startReversing = true;
          beginReversing = ros::Time::now();
          startedDetectionTimer = false;
        }
      }
      else{
        startedDetectionTimer=true;
        beginDetection = ros::Time::now();
      }
    }
    else {
      startedDetectionTimer=false;
    }

    //! @todo : 4 - When conditions are met, reverese the robot (negative 0.1 m/s x velocity)
    //! for 1 secod
    //!
    //! HINT:
    //! You will need a counter for 1 seconds and to publish the velocity
    //! Change the line below to publish your message

    if(startReversing){
      //Let's check elapsed time
      ros::Duration d = ros::Time::now() - beginReversing;
      if(d.toSec()<5){
        geometry_msgs::Twist twist;
        twist.linear.x=-0.1;
        cmd_vel_pub_.publish(twist);      // publishing the control system

      }
      else{
        geometry_msgs::Twist twist;
        cmd_vel_pub_.publish(twist);      // publishing the control system
        startReversing = false;
      }
    }
    rate_limiter.sleep();
  }
}
