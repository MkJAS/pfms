#ifndef BOGIEPOSINTERFACE_H
#define BOGIEPOSINTERFACE_H

#include <mutex>
#include <vector>
#include <chrono>
#include <algorithm> 
#include <condition_variable>
#include <atomic>
#include "dep/include/types.h"
#include "dep/include/simulator.h"

using geometry_msgs::Pose;
using geometry_msgs::Point;
using geometry_msgs::RangeBearingStamped;
using geometry_msgs::RangeVelocityStamped;
using namespace simulator;

/*!
 *  \brief     Bogpieposition Interface Class
 *  \details
 *  This interface class is used to set all the methods that need to be embodies within any subsequent derived sensor classes.
 *  \author    Joseph Seklawy
 *  \version   1.0
 *  \date      2021-10-07
 */
class BogieposInterface
{
public:

    BogieposInterface(){};


/**
 * @brief starts threads
 * 
 */
 virtual void start() = 0;


  /**
   * @brief Returns bogie positions in x,y from friendly frame of reference. Where the Point holds the
   * x and y coords, z as range and the double is the angle to the bogie
   * 
   * @return std::vector<std::pair<Point,double>> 
   */
 std::vector<std::pair<Point,double>> getbogiefriendly();



 /**
  * @brief Returns bogie positions in global frame of reference as a vector of points
  * 
  * @return std::vector<Point> 
  */
 std::vector<std::pair<Point,double>> getbogieglobal();





/**
 * @brief Gets the friendly orientation
 * 
 * @return std::pair<Point,double> 
 */
std::pair<Point,double> getfriendlyxy0();



  std::atomic<bool> globalbogiesgot_;   /** @brief test */
  std::atomic<bool> friendbogiesgot_;   
  std::atomic<bool> flying;   
  std::atomic<bool> friendly_info;   
  std::shared_ptr<Simulator> sim_;
  std::vector<std::pair<Point,double>> frnd_bogies_pos_;
  std::vector<std::pair<Point,double>> glb_bogies_pos_;
  std::vector<RangeBearingStamped> rbf_;
  //std::vector<RangeBearingStamped> rbg_;
  Point pos_friend_;
  double friend_theta_;


  std::vector<std::thread> threads_;
  std::condition_variable cv_;
  std::condition_variable FBcv_;  
  std::condition_variable GBcv_;  
  std::mutex mtx_;
  std::mutex mtx2_;

  /**
   * @brief Synchronises obtaining friendly info from sim using mutex
   * 
   */
  void getfriendinfo();

  //void calcbogiefriendly();
  /**
   * @brief Top level thread that calls other functions above to calculate both bogies x,y in
   * global and local frames of reference
   * 
   */
  void calcbogieglobalandfriendly();
  

/**
  * @brief Calculates the global x and y based off local frame of reference and location
  * 
  * @param rngbr Pair holding the range/bearing from sim function
  * @param pos_frnd position of friendly in global x,y
  * @param theta    orientation of friendly in global frame
  * @return std::pair<Point,double> 
 */
 std::pair<Point,double> calcglobalxy(Point rngbr, Point pos_frnd, double theta);



 /**
  * @brief Calculates the x,y of bogie from friendly frame of reference
  * 
  * @param rngbr Pair holding range/bearing from sim function
  * @return Point 
  */
 Point calcfrndxy(std::pair<double,double> rngbr);
};



#endif //BOGIEPOSINTERFACE_H