
#include "ros/ros.h"
#include "sample.h"


int main(int argc, char **argv)
{


    /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   *
   * The third argument below is the rosnode name
   * The name must be unique, only one node of the same name can ever register with the roscore
   * If a rosnode with same name exists, it will be terminated
   */
  ros::init(argc, argv, "robot_control");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;


  std::shared_ptr<RobotControl> rw(new RobotControl(nh));
  std::thread t(&RobotControl::seperateThread,rw);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  /**
   * Let's cleanup everything, shutdown ros and join the thread
   */
  ros::shutdown();

  return 0;
}

