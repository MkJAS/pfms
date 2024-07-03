/*! @file
 *
 *  @brief Main entry point for assignment 2.
 *
 *  TODO: Add information here
 *
 *  @author Joseph Seklawy 12578845{TODO: Your student name + id}
 *  @date 20/09/2021{TODO}
*/
#include <thread>
#include <vector>
#include <iostream>
#include "dep/include/simulator.h"
#include "bogiepos.h"
#include "control.h"
#include "bogieposAdv.h"
#include "controlAdv.h"

using geometry_msgs::Pose;
using geometry_msgs::Point;
using geometry_msgs::RangeBearingStamped;
using geometry_msgs::RangeVelocityStamped;
using namespace simulator;

// void controlThread(const std::shared_ptr<Simulator> & sim) {
//   while(true){
//     //Feed the watchdog control timer
//     double lin = sim->getFriendlyLinearVelocity();
//     double ang = sim->getFriendlyAngularVelocity();

//     sim->controlFriendly(lin, ang);
//     std::this_thread::sleep_for(std::chrono::milliseconds(50));
//   }
// }

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

  std::shared_ptr<Simulator> sim(new Simulator(true,game_mode));

  //std::cout << "Airspace Size:" << Simulator::AIRSPACE_SIZE << std::endl;


  if(game_mode == GameMode::BASIC)
  {
    Bogiepos bogiepos(sim);
    threads.push_back(sim->spawn());
    Control control(sim);
    control.start();
    bogiepos.start();
    int i = 0;
    while(true)
    {
      if(!control.getrestart())
      {
      std::vector<std::pair<Point,double>> temp1 = bogiepos.getbogieglobal();
      std::map<int,Point> temp3 = bogiepos.getmap();
      std::vector<std::pair<Point,double>> temp = bogiepos.getbogiefriendly();
      std::pair<Point,double> temp2 = bogiepos.getfriendlyxy0();
      
      control.setbogiexyth(temp, temp1,temp2,temp3);
      }
    }
    delete &control;
    delete &bogiepos;
  }
  
  if(game_mode == GameMode::ADVANCED)
  {
    std::shared_ptr<Timer> timer(new Timer());
    BogieposAdv bogieposAdv(sim,timer);
    threads.push_back(sim->spawn());
    ControlAdv controlAdv(sim,timer);
    controlAdv.start();
    bogieposAdv.start();
    int i = 0;
    while(true)
    {
      if(i == 0)
      {
      //std::vector<std::pair<Point,double>> temp1 = bogieposAdv.getbogieglobal();
      //std::vector<std::pair<Point,double>> temp = bogieposAdv.getbogiefriendly();
      //std::pair<Point,double> temp2 = bogieposAdv.getfriendlyxy0();
      std::map<int,velocity> temp3 = bogieposAdv.calcbogieVel();
      controlAdv.setbogiexyth(temp3);
      //i++;
      }
    }
    delete &controlAdv;
    delete &bogieposAdv;

  }
  
  //Join threads and end!
  for(auto & t: threads){
    t.join();
  }


  return 0;
}
