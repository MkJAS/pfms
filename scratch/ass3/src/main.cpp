#include "ros/ros.h"
#include <thread>
#include "topic_handler.h"



int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ass3");
  ros::NodeHandle nh;
  ros::NodeHandle pn("~");
  int mode;
  pn.param<int>("mode",mode,1);
  std::vector<std::thread> threads;
  //mode = 1;
  
  /**
   * Let's start seperate thread first, to do that we need to create object
   * and thereafter start the thread on teh function desired
   */
  if(mode != 2)
  {
    ROS_INFO_STREAM("Basic Mode Selected!");
    std::shared_ptr<TopicHandler> gc(new TopicHandler(nh,mode));
    threads.push_back(std::thread(&TopicHandler::seperatethread,gc));
  }
  if(mode == 2)
  {
    ROS_INFO_STREAM("Advanced Mode Selected!");
    std::shared_ptr<TopicHandler> gc(new TopicHandler(nh,mode));
    threads.push_back(std::thread(&TopicHandler::seperatethread,gc));
  }
  
  ros::spin();
  /**
   * Let's cleanup everything, shutdown ros and join the thread
   */
  ros::shutdown();
  for(auto &t: threads)
    {
        t.join();
    }
  return 0;
}

