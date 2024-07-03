#ifndef CONTROL_H
#define CONTROL_H

#include <mutex>
#include <vector>
#include <chrono>
#include <atomic>
#include <algorithm> 
#include <condition_variable>
#include "dep/include/types.h"
#include "dep/include/simulator.h"
#include "bogiepos.h"

using geometry_msgs::Pose;
using geometry_msgs::Point;
using geometry_msgs::RangeBearingStamped;
using geometry_msgs::RangeVelocityStamped;
using namespace simulator;

const double MAX_ang = 1.17;
const double MIN_ang = 0.05886;
const double MAX_vel = 1000;
const double MIN_vel = 50;
const double MAX_VA = 58.86;

struct D_A
{
  double dist;
  double angle;
};

struct XY
{
  double x;
  double y;
};




class Control
{
public:
 Control();
 Control(std::shared_ptr<Simulator> &sim);
 ~Control();
 /**
  * @brief starts threads
  * 
  */
 virtual void start();

/**
 * @brief Public getter called in main once 4 bogies have been intercepted to allow for new readings to be recieved
 * 
 * @return true 
 * @return false 
 */
 bool getrestart();

/**
 * @brief Called in the main to take data from Bogiepos class into object of this class
 * 
 * @param bogiexyth 
 * @param bogieglb 
 * @param frnd 
 */
virtual void setbogiexyth(std::vector<std::pair<Point,double>>& bogiexyth,std::vector<std::pair<Point,double>>& bogieglb, std::pair<Point,double> frnd,std::map<int,Point> map);
 

protected:
  //std::mutex mtx_;
  std::mutex mtxpos_;     /** @brief mutex for cv_ convar */ 
  //std::mutex Movemtx_;  
  std::mutex Cmdmtx_;     /** @brief command mutex, used for sending move commands via lin_vel_ and ang_vel_ */
  std::mutex Velmtx_;

  std::condition_variable cv_;  /** @brief convar used to wake up main calcpath() thread once bogie xy has been collected from bogiepos object */
  std::condition_variable Movecv_;
  std::condition_variable Velcv_; /** @brief convar used to wake up function which supplied sim with velocities which moves friendly towards bogies */

  std::vector<std::thread> threads_;
  std::atomic<bool> running_;     /** @brief bool to keep threads running when true */
  std::atomic<bool> gotbogies_;   /** @brief boolean used for cv_ predicate. Set to true when data from bogiepos class has come through */
  std::atomic<bool> gotpath_;     /** @brief boolean used to notify when a bogie order has been determined */
  std::atomic<bool> gotglobal_;
  std::atomic<bool> getvel_;      /** @brief boolean set to true once pure pursuit gamma has been calculated. Allows for combination of angular and linear velocity to be calculated to match gamma */
  std::atomic<bool> startmove_;   /** @brief once velocity from above has been calculated set to true to begin moving */
  std::shared_ptr<Simulator> sim_;
  bool restart_;

  std::vector<std::pair<Point,double>> bogiexyth_;
  std::vector<std::pair<Point,double>> bogieglb_;
  double friend_orient;
  
  
  double lin_vel_;
  double ang_vel_;
  double gamma_;
  bool base_start;

  std::map<int,Point> map_;
  std::vector<std::pair<Point,double>> POI;
  std::vector<std::pair<double,double>> POI_sort_;
  std::vector<std::vector<std::pair<D_A,int>>> BBgraph;

  std::vector<int> graph_path;   
  std::vector<int> opt_path_;
  

  void move();

  /**
   * @brief Ensures watchdog timer is satisfied
   * 
   */
  void idle();
  /**
   * @brief issues velocity commands 
   * 
   * @param lin 
   * @param ang 
   */
  void command(double lin, double ang);

  /**
   * @brief Main thread calculates bogie order via graph search
   * 
   */
  void calcpath();
  /**
   * @brief function inside calcpath() 
   * 
   * @param optpath 
   */
  void getoptimalpath(std::vector<int>& optpath);
  /**
   * @brief wraps angle between 0-PI or -PI-0
   * 
   * @param alpha 
   * @return double 
   */
  double wrapalpha(double alpha);
  /**
   * @brief Relates the bearing angle of a bogie from base to the needed friendly orientation to be pointed at the bogie
   * 
   * @param orient 
   * @param bearing 
   * @return double 
   */
  double relateOrienttoBBangle(double orient, double bearing);
  /**
   * @brief Graph search function called in optpath
   * 
   * @param node 
   * @param quickest_time 
   * @param current_orient 
   * @return int 
   */
  int graphsearch(int node, double& quickest_time,double& current_orient);
  /**
   * @brief Calculates the needed linear and angular velocites to achieve the pure pursuit gamma function
   * 
   */
  virtual void optvelocities();


std::vector<std::pair<int,int>> rangebearassociate(std::vector<RangeBearingStamped> rbs,Point friendxy);


};

#endif 