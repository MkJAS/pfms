/*! @file
 *
 *  @brief Main entry point for assignment 2.
 *
 *  TODO: Add information here
 *
 *  @author {TODO: Your student name + id}
 *  @date {TODO}
*/
#include <thread>
#include <vector>
#include <iostream>
#include "simulator.h"

using geometry_msgs::Pose;
using geometry_msgs::Point;
using geometry_msgs::RangeBearingStamped;
using geometry_msgs::RangeVelocityStamped;
using namespace simulator;

//For example purposes only, this thread attemps to get the friendly aircraft's
//(red triangle) pose every 2 seconds. It plots this pose on the simulation (blue triangle)
// via the testPose, and this pose stays on for 1 second, as per the
//'testPose()' documentation in the simualtor class.
void exampleThread(const std::shared_ptr<Simulator> & sim) {
  while(true) {
    //Get the friendly aircraft's position and orientation
    Pose pose = sim->getFriendlyPose();
    std::vector<Pose> poses;
    poses.push_back(pose);
    sim->testPose(poses);

//    std::cout << "[" << sim->elapsed() / 1000 << "s]" << std::endl;
//    std::cout << "Friendly {x, y, orientation}:"<< std::endl;
//    std::cout << "  - x: " << pose.position.x << "m" << std::endl;
//    std::cout << "  - y: " << pose.position.y << "m" << std::endl;
//    std::cout << "  - orient: " << pose.orientation*180/M_PI << " deg" << std::endl << std::endl;
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));

//! The below commented out code get's rangeVelocity and rangeBearing
//! Should not be called from this thread as these are blocking calls (they stop this thread)

//    std::vector<RangeVelocityStamped> rangeVeclocityStampedVec = sim->rangeVelocityToBogiesFromBase();
//    std::vector<RangeBearingStamped> rangeBearingStampedVec = sim->rangeBearingToBogiesFromFriendly();
//    for (auto rangeVeclocityStamped : rangeVeclocityStampedVec) {
//      std::cout << "[" << rangeVeclocityStamped.timestamp << "] [R,V]=[" <<
//                   rangeVeclocityStamped.range << ","  << rangeVeclocityStamped.velocity << "]" << std::endl;
//    }

//    for (auto rangeBearingStamped : rangeBearingStampedVec) {
//      std::cout << "[" << rangeBearingStamped.timestamp << "] [R,the]=[" <<
//                   rangeBearingStamped.range << ","  << rangeBearingStamped.bearing << "]" <<
//                   " [x,y]=[" << rangeBearingStamped.range*cos(rangeBearingStamped.bearing) << "," <<
//                                 rangeBearingStamped.range*sin(rangeBearingStamped.bearing) << "]" << std::endl;
//    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  }
}

//This thread will simply get the current velocity and feed it back into
//controlled velocity, at the designated minimum time required (watchdog time) refer
//'controlFriendly()' documentation in the simualtor class.
void controlThread(const std::shared_ptr<Simulator> & sim) {
  while(true){
    //Feed the watchdog control timer
    double lin = sim->getFriendlyLinearVelocity();
    double ang = sim->getFriendlyAngularVelocity();

    sim->controlFriendly(lin, ang);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

int main(int argc, char *argv[])
{

   GameMode game_mode = GameMode::BASIC;
   //If code is started with --advanced, it will run in advanced mode
   std::cout << "Run with: " << argv[0] << " --advanced to run in advanced mode" << std::endl;
   if(argc>1){
       if(strcmp (argv[1],"-advanced")){
           std::cout << "Advanced Mode Activated" << std::endl;
           game_mode = GameMode::ADVANCED;
       }
   }

  std::vector<std::thread> threads;

  //Create a shared pointer for the simulator class
  std::shared_ptr<Simulator> sim(new Simulator(true,game_mode));

  //Showing how to get airspace size (for example), constants arer accessible
  std::cout << "Airspace Size:" << Simulator::AIRSPACE_SIZE << std::endl;


  threads.push_back(sim->spawn());
  threads.push_back(std::thread(controlThread, sim));
  threads.push_back(std::thread(exampleThread, sim));

  //Join threads and end!
  for(auto & t: threads){
    t.join();
  }

  return 0;
}
