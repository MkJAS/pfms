#ifndef BOGIEPOS_H
#define BOGIEPOS_H

#include <mutex>
#include <vector>
#include <map>
#include <chrono>
#include <algorithm> 
#include <condition_variable>
#include <atomic>
#include "bogieposinterface.h"
#include "dep/include/types.h"
#include "dep/include/simulator.h"

using geometry_msgs::Pose;
using geometry_msgs::Point;
using geometry_msgs::RangeBearingStamped;
using geometry_msgs::RangeVelocityStamped;
using namespace simulator;


class Bogiepos
{
public:
  /**
   * @brief Construct a new Bogiepos object
   * @note Default constructor
   */
  Bogiepos();
  /**
   * @brief Construct a new Bogiepos object
   * @note The primary constructor that is called for use with the sim.
   * 
   * @param sim 
   */
  Bogiepos(std::shared_ptr<Simulator> &sim);
  /**
   * @brief Destroy the Bogiepos object
   * 
   */
 ~Bogiepos();

/**
 * @brief starts threads
 * 
 */
 virtual void start();


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


std::map<int,Point> getmap();

/**
 * @brief Calculates local bogie xy based on global coordinates
 * 
 * @param friendxy 
 * @param friendtheta 
 * @param bogiepos 
 * @return Point 
 */
Point global2local(Point friendxy, double friendtheta, Point bogiepos);

/**
 * @brief After an original call to rangebearingtobogiefromFriendly and collected a set of 4 readings,
 * a call to this function will relate a secondary set of readings from a second rangebearingtobogiefromFrienly
 * to the original
 * 
 * @param rbs 
 * @param friendxy 
 * @return std::vector<std::pair<int,int>> 
 */
std::vector<std::pair<int,int>> rangebearassociate(std::vector<RangeBearingStamped> rbs,Point friendxy);

/**
 * @brief Same as above, except with readings from the base stations
 * 
 * @param rvs 
 * @return std::vector<std::pair<int,int>> 
 */
std::vector<std::pair<int,int>> rangevelassociate(std::vector<RangeVelocityStamped> rvs);

protected:
  std::atomic<bool> globalbogiesgot_;   /** @brief boolean used for GBcv_ predicate. Triggered when global bogie xy has been calculated */
  std::atomic<bool> friendbogiesgot_;   /** @brief boolean used for FBcv_ predicate. Triggered when local bogie xy has been calculated */
  std::atomic<bool> gottimes_;          /** @brief boolean used for Tmcv_ predicate. Triggered when timestamps have been collected */
  std::atomic<bool> flying;             /** @brief boolean used to keep threads running */
  std::shared_ptr<Simulator> sim_;      /** @brief class variable of simulator object. Shared between this class and Control class */
  std::vector<std::pair<Point,double>> frnd_bogies_pos_; /** @brief vector containing bogie xy from friendly */
  std::vector<std::pair<Point,double>> glb_bogies_pos_; /** @brief vector containint bogie xy in global coord */
  std::vector<double> times_;
  std::map<int,Point> m_;               /** @brief map which stores the index from an original set of 4 bogies to their global coords. Used in association */
  std::vector<RangeBearingStamped> rbf_;
  Point pos_friend_;
  double friend_theta_;


  std::vector<std::thread> threads_;
  //std::condition_variable cv_;      
  std::condition_variable FBcv_;  /** @brief convar used to wait for bogie coords from friendly vector */
  std::condition_variable GBcv_;  /** @brief convar used to wait for bogie coords in global vector */
  std::condition_variable Tmcv_;
  std::mutex mtx_;                /** @brief mutex for glb_bogie_pos_ */
  std::mutex mtx2_;               /** @brief mutex for frnd_bogies_pos_ */
  std::mutex timemtx_;

  bool basic;

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

 /**
  * @brief Gets the timestaps of a given set of readings. Used in derived class bogieposAdv to get the times
  * for a set of rangebearing readings for the use of calculating bogie velocity
  * @return std::vector<double> 
  */
 std::vector<double> gettimes();
};

#endif // Bogiepos_H