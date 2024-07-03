
#include "sample.h"


/**
 * This sample code is provided to illustrate
 * - Subscribing to standard topics (Odometry and Laser)
 * - Publishing velocity commands
 */


RobotControl::RobotControl(ros::NodeHandle nh)
    : nh_(nh)
{
    sub1_ = nh_.subscribe("/robot_0/base_scan", 10, &RobotControl::laserCallback,this);
    sub2_ = nh_.subscribe("/robot_0/cmd_vel", 10, &RobotControl::cmdVelCallback,this);

    //Publish a velocity ... to control the robot
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel",1);
}

RobotControl::~RobotControl()
{

}


void RobotControl::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{

  //!
  //! @todo : 2 - Detect when the robot is closer than 0.15m to an obstacle
  //!


}

void RobotControl::cmdVelCallback(const geometry_msgs::TwistConstPtr & msg)
{

  //!
  //! @todo : 3 - Detect when the robot is stationary
  //!
  //! - Expect zero for linear x and angular z
  //!
  //!

}


void RobotControl::seperateThread() {
   /**
    * The below loop runs until ros is shutdown
    */

    ros::Rate rate_limiter(0.1);
    while (ros::ok()) {

      //! @todo : 4 - Check that the object is within 0.15m and the robot has been statioray for more than 3 seconds
      //!
      //! To do the check, we need to see the robot is stationary from the first time he has stopped (velocity zero)
      //! REFER : http://wiki.ros.org/roscpp/Overview/Time


      //! @todo : 5 - When conditions are met, reverese the robot (negative 0.1 m/s x velocity)
      //! for 1 secod
      //!
      //! HINT:
      //! You will need a counter for 1 seconds and to publish the velocity
      //! Change the line below to publish your message
      //cmd_vel_pub_.publish();      // publishing the control system



      rate_limiter.sleep();

    }
}




